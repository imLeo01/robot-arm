import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from PIL import Image, ImageTk
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import cv2
import os
import serial
import time
import threading

class RobotArmController:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Drawing Controller")
        self.root.geometry("1200x700")
        self.root.configure(bg="#f0f0f0")
        
        self.drawn_path_x = []
        self.drawn_path_y = []
        
        # Thiết lập biến
        self.arduino = None
        self.is_connected = False
        self.is_drawing = False
        self.stop_drawing = False
        
        # Robot parameters
        self.L1, self.L2 = 140, 120  # Chiều dài link 1 và 2 (mm)
        self.image_size = 400
        self.workspace_size = 300
        self.scale = self.workspace_size / self.image_size
        
        # Tham số mới cho việc tối ưu hóa
        self.step_size = 5.0  # Kích thước bước (mm) - càng nhỏ càng mịn
        self.step_per_mm = 10  # Số bước/mm
        self.servo_delay = 0.02  # Thời gian chờ giữa các lệnh servo (giây)
        self.motor_delay = 0.01  # Thời gian chờ giữa các lệnh động cơ (giây)
        
        # COM port and baudrate
        self.com_port = tk.StringVar(value="COM14")
        self.baudrate = tk.IntVar(value=115200)
        
        # G-code parameters
        self.gcode_list = []
        self.use_gcode = tk.BooleanVar(value=False)
        
        # Ảnh mẫu - khởi tạo trước khi gọi setup_ui
        self.available_images = self.find_image_files()
        self.image_choice = tk.StringVar(value=self.available_images[0] if self.available_images else "")
        
        # Biến lưu ảnh và đường dẫn
        self.original_image = None
        self.drawing_path = []
        self.robot_path = []
        self.current_image = None
        self.prev_angles = [0, 0]
        
        # Khởi tạo UI sau khi các biến đã được chuẩn bị
        self.setup_ui()
        
        # Cập nhật danh sách ảnh
        self.update_image_list()
        
        # Animation variables
        self.animation = None
        self.current_frame = 0
        self.drawing_thread = None
        
    def find_image_files(self):
        """Tìm tất cả các file ảnh trong thư mục hiện tại"""
        extensions = ['.png', '.jpg', '.jpeg', '.bmp', '.gif']
        image_files = []
        
        for file in os.listdir('.'):
            if any(file.lower().endswith(ext) for ext in extensions):
                image_files.append(file)
        
        return image_files if image_files else ["sample.png"]
    
    def setup_ui(self):
        """Thiết lập giao diện người dùng"""
        # Tạo style
        self.style = ttk.Style()
        self.style.configure('TButton', font=('Arial', 11))
        self.style.configure('TLabel', font=('Arial', 11))
        self.style.configure('TFrame', background="#f0f0f0")
        self.style.configure('Emergency.TButton', foreground='white', background='red', font=('Arial', 12, 'bold'))
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Chia thành 3 phần: điều khiển (trái), hiển thị (giữa), thông tin (phải)
        control_frame = ttk.LabelFrame(main_frame, text="Điều khiển", padding=10)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=5, pady=5)
        
        display_frame = ttk.LabelFrame(main_frame, text="Hiển thị", padding=10)
        display_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        info_frame = ttk.LabelFrame(main_frame, text="Thông tin", padding=10)
        info_frame.pack(side=tk.RIGHT, fill=tk.Y, padx=5, pady=5)
        
        # ===== Phần điều khiển =====
        # Kết nối Serial
        conn_frame = ttk.LabelFrame(control_frame, text="Kết nối", padding=5)
        conn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(conn_frame, text="COM Port:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Entry(conn_frame, textvariable=self.com_port, width=10).grid(row=0, column=1, sticky=tk.W, pady=2)
        
        # Nút kết nối
        self.connect_btn = ttk.Button(conn_frame, text="Kết nối", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=2, padx=5, pady=2)
        
        # Tình trạng kết nối
        self.status_var = tk.StringVar(value="Chưa kết nối")
        ttk.Label(conn_frame, textvariable=self.status_var, foreground="red").grid(row=1, column=0, columnspan=3, sticky=tk.W)
        
        # Chọn ảnh
        image_frame = ttk.LabelFrame(control_frame, text="Chọn ảnh", padding=5)
        image_frame.pack(fill=tk.X, pady=5)
        
        self.image_combo = ttk.Combobox(image_frame, textvariable=self.image_choice, state="readonly", width=25)
        self.image_combo.pack(fill=tk.X, pady=5)
        self.image_combo.bind("<<ComboboxSelected>>", self.show_image_preview)
        
        btn_frame = ttk.Frame(image_frame)
        btn_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(btn_frame, text="Tải ảnh", command=self.load_image).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Làm mới", command=self.update_image_list).pack(side=tk.LEFT, padx=5)
        
        # Cài đặt vẽ
        settings_frame = ttk.LabelFrame(control_frame, text="Cài đặt vẽ", padding=5)
        settings_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(settings_frame, text="Ngưỡng cắt:").grid(row=0, column=0, sticky=tk.W, pady=2)
        self.threshold_var = tk.IntVar(value=100)  # Giảm ngưỡng để lấy được nhiều chi tiết hơn
        threshold_slider = ttk.Scale(settings_frame, from_=0, to=255, variable=self.threshold_var, orient=tk.HORIZONTAL, length=150)
        threshold_slider.grid(row=0, column=1, padx=5, pady=2)
        threshold_slider.bind("<ButtonRelease-1>", self.process_current_image)
        
        ttk.Label(settings_frame, text="Đảo màu:").grid(row=1, column=0, sticky=tk.W, pady=2)
        self.invert_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(settings_frame, variable=self.invert_var, command=self.process_current_image).grid(row=1, column=1, sticky=tk.W, pady=2)
        
        # Cài đặt thuật toán 
        ttk.Label(settings_frame, text="Phương pháp:").grid(row=2, column=0, sticky=tk.W, pady=2)
        self.method_var = tk.StringVar(value="contour")
        method_combo = ttk.Combobox(settings_frame, textvariable=self.method_var, state="readonly", width=15, 
                                    values=["contour", "canny", "adaptive"])
        method_combo.grid(row=2, column=1, sticky=tk.W, pady=2)
        method_combo.bind("<<ComboboxSelected>>", self.process_current_image)
        
        # Chất lượng đường
        ttk.Label(settings_frame, text="Chi tiết:").grid(row=3, column=0, sticky=tk.W, pady=2)
        self.detail_var = tk.DoubleVar(value=0.5)
        detail_slider = ttk.Scale(settings_frame, from_=0.1, to=5.0, variable=self.detail_var, orient=tk.HORIZONTAL, length=150)
        detail_slider.grid(row=3, column=1, padx=5, pady=2)
        detail_slider.bind("<ButtonRelease-1>", self.process_current_image)
        
        # Thêm tùy chỉnh gốc tọa độ
        ttk.Label(settings_frame, text="Dịch X:").grid(row=4, column=0, sticky=tk.W, pady=2)
        self.offset_x = tk.DoubleVar(value=150)  # Dịch gốc tọa độ
        ttk.Entry(settings_frame, textvariable=self.offset_x, width=8).grid(row=4, column=1, padx=5, pady=2)
        
        ttk.Label(settings_frame, text="Dịch Y:").grid(row=5, column=0, sticky=tk.W, pady=2)
        self.offset_y = tk.DoubleVar(value=100)  # Dịch gốc tọa độ
        ttk.Entry(settings_frame, textvariable=self.offset_y, width=8).grid(row=5, column=1, padx=5, pady=2)
        
        ttk.Button(settings_frame, text="Áp dụng", command=self.process_current_image).grid(row=6, column=1, padx=5, pady=5)
        
        # Điều khiển vẽ
        draw_frame = ttk.LabelFrame(control_frame, text="Điều khiển vẽ", padding=5)
        draw_frame.pack(fill=tk.X, pady=5)
        
        # Thêm tùy chọn G-code
        gcode_frame = ttk.Frame(draw_frame)
        gcode_frame.pack(fill=tk.X, pady=5)
        
        ttk.Checkbutton(gcode_frame, text="Sử dụng G-code", variable=self.use_gcode).pack(side=tk.LEFT, padx=5)
        ttk.Button(gcode_frame, text="Xem G-code", command=self.show_gcode).pack(side=tk.LEFT, padx=5)
        ttk.Button(gcode_frame, text="Lưu G-code", command=self.save_gcode).pack(side=tk.LEFT, padx=5)
        
        btn_frame2 = ttk.Frame(draw_frame)
        btn_frame2.pack(fill=tk.X, pady=5)
        
        self.draw_btn = ttk.Button(btn_frame2, text="Bắt đầu vẽ", command=self.start_drawing)
        self.draw_btn.pack(side=tk.LEFT, padx=5)
        
        self.stop_btn = ttk.Button(btn_frame2, text="Dừng vẽ", command=self.stop_drawing_command, state=tk.DISABLED)
        self.stop_btn.pack(side=tk.LEFT, padx=5)
        
        # Nút khẩn cấp
        ttk.Button(draw_frame, text="DỪNG KHẨN CẤP", command=self.emergency_stop, style="Emergency.TButton").pack(fill=tk.X, pady=10)
        
        # ===== Phần hiển thị =====
        # Preview frame
        preview_frame = ttk.Frame(display_frame)
        preview_frame.pack(fill=tk.BOTH, expand=True)
        
        # Tạo frame hiển thị ảnh gốc
        self.preview_frame = ttk.LabelFrame(preview_frame, text="Ảnh gốc", padding=5)
        self.preview_frame.grid(row=0, column=0, sticky=tk.NSEW, padx=5, pady=5)
        
        self.preview_label = tk.Label(self.preview_frame, bg="white")
        self.preview_label.pack(fill=tk.BOTH, expand=True)
        
        # Tạo frame hiển thị đường nét
        self.path_frame = ttk.LabelFrame(preview_frame, text="Đường nét trích xuất", padding=5)
        self.path_frame.grid(row=0, column=1, sticky=tk.NSEW, padx=5, pady=5)
        
        self.fig_path = plt.Figure(figsize=(4, 4), dpi=100)
        self.ax_path = self.fig_path.add_subplot(111)
        self.canvas_path = FigureCanvasTkAgg(self.fig_path, master=self.path_frame)
        self.canvas_path.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Tạo frame hiển thị mô phỏng
        self.robot_frame = ttk.LabelFrame(preview_frame, text="Mô phỏng robot", padding=5)
        self.robot_frame.grid(row=1, column=0, columnspan=2, sticky=tk.NSEW, padx=5, pady=5)
        
        self.fig_robot = plt.Figure(figsize=(8, 5), dpi=100)
        self.ax_robot = self.fig_robot.add_subplot(111)
        self.canvas_robot = FigureCanvasTkAgg(self.fig_robot, master=self.robot_frame)
        self.canvas_robot.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Configure grid
        preview_frame.columnconfigure(0, weight=1)
        preview_frame.columnconfigure(1, weight=1)
        preview_frame.rowconfigure(0, weight=1)
        preview_frame.rowconfigure(1, weight=1)
        
        # ===== Phần thông tin =====
        # Thông tin robot
        robot_info_frame = ttk.LabelFrame(info_frame, text="Thông tin robot", padding=5)
        robot_info_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(robot_info_frame, text="Link 1:").grid(row=0, column=0, sticky=tk.W, pady=2)
        ttk.Label(robot_info_frame, text=f"{self.L1} mm").grid(row=0, column=1, sticky=tk.W, pady=2)
        
        ttk.Label(robot_info_frame, text="Link 2:").grid(row=1, column=0, sticky=tk.W, pady=2)
        ttk.Label(robot_info_frame, text=f"{self.L2} mm").grid(row=1, column=1, sticky=tk.W, pady=2)
        
        ttk.Label(robot_info_frame, text="Tỷ lệ bước/mm:").grid(row=2, column=0, sticky=tk.W, pady=2)
        ttk.Label(robot_info_frame, text=f"{self.step_per_mm}").grid(row=2, column=1, sticky=tk.W, pady=2)
        
        # Thông tin vẽ
        drawing_info_frame = ttk.LabelFrame(info_frame, text="Thông tin vẽ", padding=5)
        drawing_info_frame.pack(fill=tk.X, pady=5)
        
        self.points_var = tk.StringVar(value="Số điểm: 0")
        ttk.Label(drawing_info_frame, textvariable=self.points_var).pack(anchor=tk.W, pady=2)
        
        self.progress_var = tk.StringVar(value="Tiến độ: 0%")
        ttk.Label(drawing_info_frame, textvariable=self.progress_var).pack(anchor=tk.W, pady=2)
        
        self.progress = ttk.Progressbar(drawing_info_frame, orient=tk.HORIZONTAL, length=200, mode='determinate')
        self.progress.pack(fill=tk.X, pady=5)
        
        # Góc hiện tại
        angle_frame = ttk.LabelFrame(info_frame, text="Góc hiện tại", padding=5)
        angle_frame.pack(fill=tk.X, pady=5)
        
        self.theta1_var = tk.StringVar(value="θ1: 0.0°")
        ttk.Label(angle_frame, textvariable=self.theta1_var).pack(anchor=tk.W, pady=2)
        
        test_frame = ttk.LabelFrame(control_frame, text="Diagnostics", padding=5)
        test_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(test_frame, text="Test Motors", command=self.test_motors).pack(fill=tk.X, pady=5)
        self.theta2_var = tk.StringVar(value="θ2: 0.0°")
        ttk.Label(angle_frame, textvariable=self.theta2_var).pack(anchor=tk.W, pady=2)
        
        # Hướng dẫn
        guide_frame = ttk.LabelFrame(info_frame, text="Hướng dẫn", padding=5)
        guide_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        guide_text = """1. Kết nối với cổng COM
2. Chọn ảnh để vẽ
3. Điều chỉnh ngưỡng, đảo màu và chọn phương pháp trích xuất
4. Điều chỉnh mức chi tiết phù hợp
5. Tùy chọn: Xem/Lưu G-code
6. Nhấn 'Bắt đầu vẽ' để vẽ ảnh
7. Có thể dừng quá trình vẽ bất cứ lúc nào
8. Nút DỪNG KHẨN CẤP sẽ dừng robot ngay lập tức

Chú ý: Đảm bảo robot đã ở vị trí home trước khi vẽ."""
        
        guide_label = ttk.Label(guide_frame, text=guide_text, wraplength=250, justify=tk.LEFT)
        guide_label.pack(fill=tk.BOTH, expand=True)
        
        # Footer
        footer_frame = ttk.Frame(self.root)
        footer_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(footer_frame, text="© 2025 Robot Drawing Controller - HCMUTE", font=('Arial', 9)).pack(side=tk.RIGHT)
    
    def update_image_list(self):
        """Cập nhật danh sách ảnh"""
        self.available_images = self.find_image_files()
        self.image_combo['values'] = self.available_images
        
        if self.available_images and not self.image_choice.get():
            self.image_choice.set(self.available_images[0])
            self.show_image_preview()
    
    def load_image(self):
        """Mở hộp thoại chọn file ảnh"""
        file_path = filedialog.askopenfilename(
            title="Chọn ảnh",
            filetypes=[("Image files", "*.png *.jpg *.jpeg *.bmp *.gif")]
        )
        
        if file_path:
            # Lấy tên file
            file_name = os.path.basename(file_path)
            
            # Nếu file không trong thư mục hiện tại, sao chép vào
            if not os.path.exists(file_name):
                # Đọc file ảnh
                img = Image.open(file_path)
                # Lưu vào thư mục hiện tại
                img.save(file_name)
            
            # Cập nhật danh sách ảnh
            self.update_image_list()
            
            # Chọn file mới
            self.image_choice.set(file_name)
            self.show_image_preview()
    
    def show_image_preview(self, event=None):
        """Hiển thị xem trước ảnh"""
        image_path = self.image_choice.get()
        
        if image_path and os.path.exists(image_path):
            try:
                # Hiển thị ảnh gốc
                img = Image.open(image_path).convert("L")
                img = img.resize((250, 250), Image.LANCZOS)
                img_tk = ImageTk.PhotoImage(img)
                self.preview_label.configure(image=img_tk)
                self.preview_label.image = img_tk
                
                # Lưu ảnh hiện tại
                self.current_image = image_path
                
                # Xử lý ảnh
                self.process_current_image()
            except Exception as e:
                messagebox.showerror("Lỗi", f"Không thể mở ảnh: {str(e)}")
                self.preview_label.configure(image=None, text="Lỗi hiển thị ảnh")
        else:
            self.preview_label.configure(image=None, text="Không tìm thấy ảnh.")
    
    def process_current_image(self, event=None):
        """Xử lý ảnh hiện tại để trích xuất đường nét"""
        if not self.current_image or not os.path.exists(self.current_image):
            return
        
        try:
            # Trích xuất đường nét từ ảnh
            threshold = self.threshold_var.get()
            invert = self.invert_var.get()
            method = self.method_var.get()
            detail_level = self.detail_var.get()
            
            self.original_image, self.drawing_path = self.extract_drawing_path(
                self.current_image, threshold, invert, method, detail_level
            )
            
            # Tối ưu đường đi
            self.drawing_path = self.optimize_path(self.drawing_path)
            
            # Chuyển sang tọa độ robot
            _, self.robot_path = self.convert_to_robot_coords(self.drawing_path)
            
            # Tạo G-code
            self.generate_gcode()
            
            # Hiển thị đường nét
            self.show_drawing_path()
            
            # Cập nhật thông tin
            self.points_var.set(f"Số điểm: {len(self.robot_path)}")
            self.progress_var.set("Tiến độ: 0%")
            self.progress['value'] = 0
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể xử lý ảnh: {str(e)}")
    
    def extract_drawing_path(self, image_path, threshold=128, invert=True, method="contour", detail_level=1.0):
        """Trích xuất đường nét từ ảnh với nhiều phương pháp khác nhau"""
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        
        if img is None:
            raise ValueError(f"Không thể đọc ảnh: {image_path}")
        
        # Áp dụng bộ lọc khử nhiễu (làm mịn)
        img_blur = cv2.GaussianBlur(img, (5, 5), 0)
        
        # Áp dụng bộ lọc tăng cường cạnh trước khi phân ngưỡng
        img_enhanced = cv2.Laplacian(img_blur, cv2.CV_8U, ksize=3)
        img_blur = cv2.addWeighted(img_blur, 0.7, img_enhanced, 0.3, 0)
        
        drawing_path = []
        
        # Detail level ảnh hưởng đến epsilon trong approxPolyDP
        epsilon_factor = 0.03 / detail_level  # Càng nhỏ càng chi tiết
        
        if method == "contour":
            # Phương pháp ngưỡng nhị phân và tìm đường viền
            if invert:
                _, binary = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY_INV)
            else:
                _, binary = cv2.threshold(img_blur, threshold, 255, cv2.THRESH_BINARY)
            
            # Áp dụng phép toán hình thái học để nối các đường gần nhau
            kernel = np.ones((2, 2), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
            
            # Tìm tất cả các contour, bao gồm cả contour bên trong
            contours, hierarchy = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
        elif method == "canny":
            # Phương pháp dùng Canny Edge Detection
            edges = cv2.Canny(img_blur, threshold, threshold * 2)
            # Áp dụng phép giãn nở để kết nối các cạnh bị đứt
            kernel = np.ones((2, 2), np.uint8)
            edges = cv2.dilate(edges, kernel, iterations=1)
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            
        elif method == "adaptive":
            # Phương pháp ngưỡng thích ứng
            block_size = 11  # Kích thước block - cần là số lẻ
            C = 2  # Hằng số hiệu chỉnh
            
            if invert:
                binary = cv2.adaptiveThreshold(img_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                            cv2.THRESH_BINARY_INV, block_size, C)
            else:
                binary = cv2.adaptiveThreshold(img_blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                            cv2.THRESH_BINARY, block_size, C)
            
            # Áp dụng phép toán hình thái học để nối các đường gần nhau
            kernel = np.ones((2, 2), np.uint8)
            binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
            
            contours, hierarchy = cv2.findContours(binary, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        
        else:
            raise ValueError(f"Phương pháp không hợp lệ: {method}")
        
        # Sắp xếp contour theo kích thước (từ lớn đến nhỏ)
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        
        # Tạo danh sách điểm từ contours, ưu tiên contour lớn trước
        for contour in contours:
            # Bỏ qua contour quá nhỏ
            if cv2.contourArea(contour) < 5:  # Giảm kích thước tối thiểu để bắt nhiều chi tiết hơn
                continue
            
            # Độ chi tiết của đường viền phụ thuộc vào detail_level
            epsilon = epsilon_factor * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            
            # Thêm điểm đánh dấu đường viền mới
            if len(drawing_path) > 0:
                drawing_path.append((-1, -1))  # Đánh dấu đường viền mới
            
            # Thêm các điểm từ đường viền này
            for point in approx:
                x, y = point[0]
                drawing_path.append((x, y))
        
        # Đảm bảo có đường nét để vẽ
        if not drawing_path:
            # Nếu không tìm thấy đường viền với ngưỡng hiện tại, thử lại với ngưỡng thấp hơn
            if method == "contour" and threshold > 50:
                print("Thử lại với ngưỡng thấp hơn:", threshold - 30)
                return self.extract_drawing_path(image_path, threshold - 30, invert, method, detail_level)
            else:
                raise ValueError("Không thể trích xuất đường nét từ ảnh. Hãy thử điều chỉnh ngưỡng hoặc phương pháp.")
        
        return img, drawing_path
    
    def optimize_path(self, drawing_path):
        """Tối ưu đường đi để có chuyển động mượt hơn"""
        if not drawing_path:
            return []
            
        optimized_path = []
        current_contour = []
        
        for point in drawing_path:
            if point == (-1, -1):  # Đánh dấu đường viền mới
                if current_contour:
                    # Đóng đường viền (kết nối điểm đầu và cuối)
                    if len(current_contour) > 1 and current_contour[0] != current_contour[-1]:
                        current_contour.append(current_contour[0])
                    
                    # Thêm vào đường đi tối ưu
                    optimized_path.extend(current_contour)
                    
                    # Đánh dấu bắt đầu đường viền mới
                    if optimized_path:
                        optimized_path.append((-1, -1))
                        
                current_contour = []
            else:
                current_contour.append(point)
        
        # Xử lý contour cuối cùng
        if current_contour:
            # Đóng đường viền nếu cần
            if len(current_contour) > 1 and current_contour[0] != current_contour[-1]:
                current_contour.append(current_contour[0])
            
            # Thêm vào đường đi tối ưu
            optimized_path.extend(current_contour)
        
        # Nội suy thêm điểm để đường đi mượt hơn
        interpolated_path = []
        for i in range(len(optimized_path)):
            if optimized_path[i] == (-1, -1):
                interpolated_path.append((-1, -1))
                continue
                
            # Thêm điểm hiện tại
            interpolated_path.append(optimized_path[i])
            # Nội suy thêm điểm giữa điểm hiện tại và điểm tiếp theo
            if i < len(optimized_path) - 1 and optimized_path[i+1] != (-1, -1) and optimized_path[i] != (-1, -1):
                x1, y1 = optimized_path[i]
                x2, y2 = optimized_path[i+1]
                
                # Tính khoảng cách giữa 2 điểm
                distance = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                
                # Nếu khoảng cách đủ lớn, thêm điểm ở giữa
                step = self.step_size
                if distance > step * 2:
                    num_points = int(distance / step) - 1
                    for j in range(1, num_points + 1):
                        ratio = j / (num_points + 1)
                        x = x1 + (x2 - x1) * ratio
                        y = y1 + (y2 - y1) * ratio
                        interpolated_path.append((x, y))
        
        return interpolated_path
    
    def convert_to_robot_coords(self, drawing_path):
        """Chuyển đường nét từ tọa độ ảnh sang tọa độ robot với việc xử lý nhấc/hạ bút tốt hơn"""
        robot_coords = []
        
        # Tìm kích thước ảnh
        if self.original_image is not None:
            height, width = self.original_image.shape
        else:
            height, width = self.image_size, self.image_size
        
        # Tỷ lệ chuyển đổi
        scale = self.workspace_size / max(width, height) * 0.8  # thu nhỏ hình một chút
        
        # Lấy offset từ giao diện
        offset_x = self.offset_x.get()
        offset_y = self.offset_y.get()
        
        current_segments = []
        current_segment = []
        
        # Tách các đoạn (khoảng cách -1, -1)
        for point in drawing_path:
            if point == (-1, -1):
                if current_segment:
                    current_segments.append(current_segment)
                    current_segment = []
            else:
                current_segment.append(point)
        
        # Thêm đoạn cuối cùng nếu có
        if current_segment:
            current_segments.append(current_segment)
        
        # Nhấc bút lên ở vị trí bắt đầu
        first_segment = True
        
        # Tạo đường đi robot với việc nhấc/hạ bút rõ ràng
        for segment in current_segments:
            if not segment:
                continue
            
            # Chuyển đổi các điểm trong segment sang tọa độ robot
            robot_segment = []
            for point in segment:
                x_img, y_img = point
                x_robot = (x_img - width/2) * scale + offset_x
                y_robot = (height/2 - y_img) * scale + offset_y
                robot_segment.append((x_robot, y_robot))
            
            # Thêm lệnh nhấc bút và di chuyển đến điểm đầu tiên của segment
            first_x, first_y = robot_segment[0]
            
            # Nhấc bút lên trước khi di chuyển đến vị trí mới
            robot_coords.append((first_x, first_y, 0))  # Di chuyển với bút lên
            
            # Hạ bút xuống tại vị trí đầu của segment
            robot_coords.append((first_x, first_y, 1))  # Hạ bút xuống
            
            # Thêm các điểm còn lại trong segment (đều với bút hạ xuống)
            for x, y in robot_segment[1:]:
                robot_coords.append((x, y, 1))  # Vẽ với bút xuống
            
            # Nhấc bút lên tại điểm cuối của segment
            last_x, last_y = robot_segment[-1]
            robot_coords.append((last_x, last_y, 0))  # Nhấc bút lên
        
        return self.original_image, robot_coords
    
    def generate_gcode(self):
        """Tạo G-code từ đường đi robot"""
        gcode = []
        
        # Thêm tiêu đề và các lệnh khởi tạo
        gcode.append("; Generated G-code for drawing")
        gcode.append("; Created by Robot Drawing Controller")
        gcode.append("G21 ; Set units to millimeters")
        gcode.append("G90 ; Use absolute coordinates")
        gcode.append("G92 X0 Y0 Z0 ; Reset position")
        gcode.append("G0 Z5 ; Lift pen to safe height")
        gcode.append("G0 X0 Y0 ; Move to home position")
        
        # Feedrate (tốc độ di chuyển)
        travel_speed = 3000  # mm/min khi di chuyển không vẽ
        drawing_speed = 4000  # mm/min khi vẽ
        
        pen_up_position = 5  # mm
        pen_down_position = 0  # mm
        
        prev_pen_state = 0  # Bắt đầu với bút lên
        
        for x, y, pen_state in self.robot_path:
            # Nếu trạng thái bút thay đổi
            if pen_state != prev_pen_state:
                if pen_state == 1:  # Hạ bút xuống
                    gcode.append(f"G0 Z{pen_down_position} ; Lower pen")
                    gcode.append(f"G1 F{drawing_speed} ; Set drawing speed")
                else:  # Nâng bút lên
                    gcode.append(f"G0 Z{pen_up_position} ; Lift pen")
                    gcode.append(f"G0 F{travel_speed} ; Set travel speed")
                prev_pen_state = pen_state
            
            # Lệnh di chuyển
            if pen_state == 1:
                # Bút xuống - vẽ đường
                gcode.append(f"G1 X{x:.2f} Y{y:.2f}")
            else:
                # Bút lên - di chuyển
                gcode.append(f"G0 X{x:.2f} Y{y:.2f}")
        
        # Kết thúc với bút lên và về home
        gcode.append("G0 Z5 ; Lift pen to safe height")
        gcode.append("G0 X0 Y0 ; Return to home position")
        
        self.gcode_list = gcode
        return gcode
    
    def save_gcode(self):
        """Lưu G-code vào file"""
        if not self.gcode_list:
            messagebox.showinfo("Thông báo", "Chưa có G-code được tạo. Vui lòng xử lý ảnh trước.")
            return
        
        # Mở hộp thoại lưu file
        file_path = filedialog.asksaveasfilename(
            title="Lưu G-code",
            defaultextension=".gcode",
            filetypes=[("G-code files", "*.gcode"), ("Text files", "*.txt"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                with open(file_path, 'w') as f:
                    f.write("\n".join(self.gcode_list))
                messagebox.showinfo("Thành công", f"Đã lưu G-code vào file {file_path}")
            except Exception as e:
                messagebox.showerror("Lỗi", f"Không thể lưu file: {str(e)}")
    
    def show_gcode(self):
        """Hiển thị G-code đã tạo"""
        if not self.gcode_list:
            messagebox.showinfo("Thông báo", "Chưa có G-code được tạo. Vui lòng xử lý ảnh trước.")
            return
        
        # Tạo cửa sổ mới để hiển thị G-code
        gcode_window = tk.Toplevel(self.root)
        gcode_window.title("G-code Preview")
        gcode_window.geometry("600x500")
        
        # Tạo text area để hiển thị
        text_frame = ttk.Frame(gcode_window, padding=10)
        text_frame.pack(fill=tk.BOTH, expand=True)
        
        gcode_text = tk.Text(text_frame, wrap=tk.NONE)
        gcode_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        # Thêm scrollbar
        scrollbar = ttk.Scrollbar(text_frame, command=gcode_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        gcode_text.config(yscrollcommand=scrollbar.set)
        
        # Thêm nội dung G-code
        gcode_text.insert(tk.END, "\n".join(self.gcode_list))
        
        # Nút đóng
        ttk.Button(gcode_window, text="Đóng", command=gcode_window.destroy).pack(pady=10)
    
    def show_drawing_path(self):
        """Hiển thị đường nét trích xuất"""
        self.ax_path.clear()
        
        if not self.drawing_path:
            return
        
        # Phân tách các đường viền để vẽ
        segments = []
        current_segment = []
        
        for point in self.drawing_path:
            if point == (-1, -1):
                if current_segment:
                    segments.append(current_segment)
                    current_segment = []
            else:
                current_segment.append(point)
        
        # Thêm đoạn cuối cùng nếu có
        if current_segment:
            segments.append(current_segment)
        
        # Vẽ từng đoạn với màu khác nhau
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'k']
        
        for i, segment in enumerate(segments):
            color = colors[i % len(colors)]
            
            x_vals = [p[0] for p in segment]
            y_vals = [p[1] for p in segment]
            
            self.ax_path.plot(x_vals, y_vals, f'{color}-', linewidth=1.2)
        
        self.ax_path.set_aspect('equal')
        self.ax_path.axis('off')
        
        self.canvas_path.draw()
    
    def simulate_robot_arm(self, robot_coords, frame_idx):
        """Mô phỏng cánh tay robot và hiển thị quá trình vẽ"""
        self.ax_robot.clear()
        
        if not robot_coords or frame_idx >= len(robot_coords):
            return
        
        # Vẽ lưới nhẹ làm nền
        self.ax_robot.grid(True, linestyle='--', alpha=0.3)
        
        # Vẽ đường đã vẽ (chỉ những điểm có pen_state = 1)
        path_x = []
        path_y = []
        
        # Tách đường thành nhiều đoạn dựa vào trạng thái bút
        drawing_segments = []
        current_segment = []
        pen_down = False
        
        for i, (x, y, pen) in enumerate(robot_coords[:frame_idx+1]):
            if pen == 1:  # Bút đang hạ xuống
                if not pen_down:  # Bút vừa được hạ xuống
                    pen_down = True
                    current_segment = [(x, y)]
                else:  # Bút tiếp tục vẽ
                    current_segment.append((x, y))
                    # Thêm vào đường đã vẽ
                    self.drawn_path_x.append(x)
                    self.drawn_path_y.append(y)
            else:  # Bút đang nhấc lên
                if pen_down:  # Bút vừa được nhấc lên
                    pen_down = False
                    if len(current_segment) > 1:
                        drawing_segments.append(current_segment)
                    current_segment = []
        
        # Thêm đoạn cuối cùng nếu bút vẫn đang hạ xuống
        if pen_down and len(current_segment) > 1:
            drawing_segments.append(current_segment)
        
        # Vẽ từng đoạn với màu khác nhau
        colors = ['green', 'blue', 'red', 'purple', 'orange', 'teal']
        for i, segment in enumerate(drawing_segments):
            color = colors[i % len(colors)]
            segment_x = [p[0] for p in segment]
            segment_y = [p[1] for p in segment]
            self.ax_robot.plot(segment_x, segment_y, color=color, linewidth=1.5, zorder=2)
        
        # Lấy tọa độ và trạng thái bút hiện tại
        x, y, pen = robot_coords[frame_idx]
        
        # Tính góc từ tọa độ bằng động học ngược
        angles = self.inverse_kinematics(x, y)
        if not angles:
            return
        
        theta1, theta2 = angles
        self.theta1_var.set(f"θ1: {theta1:.1f}°")
        self.theta2_var.set(f"θ2: {theta2:.1f}°")
        
        # Tính toán vị trí điểm nối và điểm cuối
        x1 = self.L1 * np.cos(np.radians(theta1))
        y1 = self.L1 * np.sin(np.radians(theta1))
        
        x2 = x1 + self.L2 * np.cos(np.radians(theta1 + theta2))
        y2 = y1 + self.L2 * np.sin(np.radians(theta1 + theta2))
        
        # Vẽ cánh tay robot với độ dày và màu sắc rõ ràng hơn
        self.ax_robot.plot([0, x1], [0, y1], 'ro-', linewidth=4, markersize=8, zorder=3)  # Link 1
        self.ax_robot.plot([x1, x2], [y1, y2], 'bo-', linewidth=4, markersize=8, zorder=3)  # Link 2
        
        # Hiển thị đầu bút với màu sắc khác nhau tùy trạng thái
        pen_color = 'red' if pen == 0 else 'green'
        self.ax_robot.scatter([x2], [y2], s=100, color=pen_color, edgecolor='black', zorder=4)
        
        # Hiển thị trạng thái bút
        pen_status = "Đang vẽ" if pen == 1 else "Nhấc lên"
        self.ax_robot.set_title(f"Mô phỏng robot - Điểm {frame_idx+1}/{len(robot_coords)} - Bút: {pen_status}")
        
        # Vẽ vùng làm việc
        circle = plt.Circle((0, 0), self.L1 + self.L2, fill=False, color='gray', linestyle='--', alpha=0.5)
        self.ax_robot.add_patch(circle)
        
        if abs(self.L1 - self.L2) > 1:
            inner_circle = plt.Circle((0, 0), abs(self.L1 - self.L2), fill=False, color='gray', linestyle='--', alpha=0.5)
            self.ax_robot.add_patch(inner_circle)
        
        # Vẽ trục tọa độ
        self.ax_robot.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        self.ax_robot.axvline(x=0, color='k', linestyle='-', alpha=0.3)
        
        # Thiết lập giới hạn trục để nhìn thấy toàn bộ vùng làm việc
        limit = self.L1 + self.L2 + 50
        self.ax_robot.set_xlim(-limit, limit)
        self.ax_robot.set_ylim(-limit, limit)
        self.ax_robot.set_aspect('equal')
        
        # Cập nhật canvas
        self.canvas_robot.draw()
    
    def inverse_kinematics(self, x, y):
        """Tính động học ngược (x, y) -> (theta1, theta2)"""
        d = (x**2 + y**2 - self.L1**2 - self.L2**2) / (2 * self.L1 * self.L2)
        
        if abs(d) > 1:  # Điểm ngoài tầm với
            return None
            
        # Sử dụng elbow-down configuration để được đường đi mượt hơn
        theta2 = -np.arccos(d)  # Dấu trừ để cánh tay hướng xuống
        theta1 = np.arctan2(y, x) - np.arctan2(self.L2 * np.sin(theta2), self.L1 + self.L2 * np.cos(theta2))
        
        # Chuyển từ radian sang độ
        return np.degrees(theta1), np.degrees(theta2)
    
    def toggle_connection(self):
        """Kết nối/ngắt kết nối với Arduino"""
        if self.is_connected:
            if self.arduino:
                self.arduino.close()
                self.arduino = None
            self.is_connected = False
            self.connect_btn.config(text="Kết nối")
            self.status_var.set("Đã ngắt kết nối")
        else:
            port = self.com_port.get()
            baudrate = self.baudrate.get()
            try:
                self.arduino = serial.Serial(port, baudrate, timeout=1)
                time.sleep(2)  # Chờ Arduino sẵn sàng
                self.is_connected = True
                self.connect_btn.config(text="Ngắt kết nối")
                self.status_var.set(f"Đã kết nối với {port}")

                # Đặt động cơ về vị trí ban đầu
                self.send_command("HOME")
                self.last_dx = 0
                self.last_dy = 0
            except Exception as e:
                messagebox.showerror("Lỗi kết nối", f"Không thể kết nối với Arduino: {str(e)}")
    def send_command(self, command):
        """Gửi lệnh đến Arduino"""
        if not self.is_connected or not self.arduino:
            messagebox.showwarning("Cảnh báo", "Chưa kết nối với Arduino!")
            return False
            
        try:
            # Đảm bảo lệnh kết thúc bằng ký tự xuống dòng
            if not command.endswith('\n'):
                command += '\n'
                
            self.arduino.write(command.encode())
            time.sleep(0.01)  # Chờ một chút để Arduino xử lý
            
            # Đọc phản hồi
            response = self.arduino.readline().decode().strip()
            print(f"Arduino response: {response}")
            
            return True
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể gửi lệnh: {str(e)}")
            return False
            
    def send_gcode(self, gcode_line):
        """Gửi lệnh G-code đến máy CNC"""
        if not self.is_connected or not self.arduino:
            messagebox.showwarning("Cảnh báo", "Chưa kết nối với máy CNC!")
            return False
            
        try:
            # Đảm bảo lệnh kết thúc bằng ký tự xuống dòng
            if not gcode_line.endswith('\n'):
                gcode_line += '\n'
                
            # Gửi G-code
            self.arduino.write(gcode_line.encode())
            time.sleep(0.01)  # Chờ một chút để Arduino xử lý
            
            # Đọc phản hồi
            response = self.arduino.readline().decode().strip()
            print(f"G-code response: {response}")
                
            return True
        except Exception as e:
            messagebox.showerror("Lỗi", f"Không thể gửi G-code: {str(e)}")
            return False
    
    def test_motors(self):
        """Test basic motor functionality"""
        if not self.is_connected:
            messagebox.showwarning("Warning", "Please connect to Arduino first!")
            return
        
        try:
            # Send HOME command first
            self.send_command("HOME")
            time.sleep(1)  # Wait for home operation to complete
            
            # Test pen up/down
            self.send_command("PU")
            time.sleep(0.5)
            self.send_command("PD")
            time.sleep(0.5)
            self.send_command("PU")
            time.sleep(0.5)
            
            # Test simple angle movements
            test_angles = [
                (10, 10),   # Small movement
                (30, 15),   # Medium movement
                (45, 30),   # Larger movement
                (0, 0)      # Back to home
            ]
            
            for theta1, theta2 in test_angles:
                command = f"GOTO {theta1} {theta2}"
                print(f"Testing movement to {theta1}°, {theta2}°")
                self.send_command(command)
                time.sleep(1)  # Wait for movement to complete
                
                # Request status to verify position
                self.send_command("STATUS")
                time.sleep(0.5)
        
            messagebox.showinfo("Test Complete", "Motor test sequence completed.\nCheck console for details.")
        except Exception as e:
            messagebox.showerror("Test Error", f"Error during motor test: {str(e)}")

    def move_physical_robot(self, prev_angles, theta1, theta2, pen):
        """Điều khiển robot thực tế với chuyển động mượt mà và đồng bộ với servo"""
        if not self.is_connected or not self.arduino:
            return False
        
        try:
            # Make sure Arduino's buffer isn't overwhelmed
            if self.arduino.in_waiting > 0:
                # Read and clear the buffer
                response = self.arduino.read(self.arduino.in_waiting)
                print(f"Buffer cleared: {response}")
            
            # Handle pen state first
            # If changing from drawing to lifting, lift pen before moving
            if hasattr(self, 'current_pen') and self.current_pen == 1 and pen == 0:
                self.send_command("PU")  # Lift pen first
                time.sleep(self.servo_delay * 3)  # Wait longer to ensure pen is lifted
                self.current_pen = 0
            
            # Direct angle command - the Arduino code expects angles directly
            command = f"GOTO {theta1:.2f} {theta2:.2f}"
            success = self.send_command(command)
            
            # Add a delay to ensure the command is processed
            time.sleep(self.motor_delay * 2)
            
            # Wait for a response to confirm movement is complete
            if success:
                # Try to get a response
                response = ""
                start_time = time.time()
                while time.time() - start_time < 1.0:  # Timeout after 1 second
                    if self.arduino.in_waiting > 0:
                        response += self.arduino.readline().decode().strip()
                        if "Moved to angle" in response:
                            break
                    time.sleep(0.1)
                
                if not response:
                    print("Warning: No movement confirmation received")
            
            # If changing from lifting to drawing, lower pen after movement
            if (not hasattr(self, 'current_pen') or self.current_pen == 0) and pen == 1:
                self.send_command("PD")  # Lower pen after reaching position
                time.sleep(self.servo_delay * 3)  # Wait longer to ensure pen is down
                self.current_pen = 1
                
            return True
        except Exception as e:
            print(f"Error controlling robot: {str(e)}")
            return False
    
    def start_drawing(self):
        """Bắt đầu quá trình vẽ"""
        # Kiểm tra G-code hoặc kết nối
        if self.use_gcode.get():
            if not self.robot_path:
                messagebox.showwarning("Cảnh báo", "Không có đường nét để vẽ. Vui lòng chọn ảnh!")
                return
                
            # Hỏi người dùng xác nhận
            result = messagebox.askquestion("Xác nhận", "Bắt đầu mô phỏng quá trình vẽ với G-code?")
            if result != 'yes':
                return
                
            # Cập nhật trạng thái
            self.is_drawing = True
            self.stop_drawing = False
            
            # Cập nhật nút
            self.draw_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            
            self.prev_angles = [0, 0]

            # Bắt đầu vẽ trong một thread riêng biệt
            self.drawing_thread = threading.Thread(target=self.drawing_process)
            self.drawing_thread.daemon = True
            self.drawing_thread.start()
            
        else:
            if not self.is_connected:
                messagebox.showwarning("Cảnh báo", "Vui lòng kết nối với Arduino trước khi vẽ!")
                return
            
            if not self.robot_path:
                messagebox.showwarning("Cảnh báo", "Không có đường nét để vẽ. Vui lòng chọn ảnh!")
                return
            
            # Kiểm tra xem đang vẽ không
            if self.is_drawing:
                messagebox.showinfo("Thông báo", "Đang trong quá trình vẽ!")
                return
            
            # Hỏi người dùng xác nhận
            result = messagebox.askquestion("Xác nhận", "Bắt đầu quá trình vẽ? Đảm bảo robot đã ở vị trí home.")
            if result != 'yes':
                return
            
            # Cập nhật trạng thái
            self.is_drawing = True
            self.stop_drawing = False
            
            # Cập nhật nút
            self.draw_btn.config(state=tk.DISABLED)
            self.stop_btn.config(state=tk.NORMAL)
            
            # Reset góc hiện tại
            self.prev_angles = [0, 0]
            
            # Bắt đầu vẽ trong một thread riêng biệt
            self.drawing_thread = threading.Thread(target=self.drawing_process)
            self.drawing_thread.daemon = True
            self.drawing_thread.start()
    
    def gcode_simulation_process(self):
        """Mô phỏng quá trình vẽ sử dụng G-code"""
        try:
            total_points = len(self.robot_path)
            print(f"Bắt đầu mô phỏng {total_points} điểm với G-code")
            
            # Lặp qua từng điểm trong đường đi robot
            for i, (x, y, pen) in enumerate(self.robot_path):
                # Kiểm tra dừng
                if self.stop_drawing:
                    break
                
                # Hiển thị mô phỏng
                self.root.after(0, lambda idx=i: self.simulate_robot_arm(self.robot_path, idx))
                
                # Tính toán góc
                angles = self.inverse_kinematics(x, y)
                
                if not angles:
                    print(f"Bỏ qua điểm {i}: Ngoài tầm với ({x}, {y})")
                    continue
                
                theta1, theta2 = angles
                self.theta1_var.set(f"θ1: {theta1:.1f}°")
                self.theta2_var.set(f"θ2: {theta2:.1f}°")
                
                # Cập nhật tiến độ
                progress = (i + 1) / total_points * 100
                self.root.after(0, lambda p=progress: self.update_progress(p))
                
                # Chờ một chút giữa các điểm
                time.sleep(0.05)
            
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Lỗi", f"Lỗi trong quá trình mô phỏng: {str(e)}"))
        finally:
            # Cập nhật trạng thái
            self.is_drawing = False
            self.root.after(0, self.reset_drawing_ui)
    
    def drawing_process(self):
        """Quá trình vẽ (chạy trong thread riêng) với animation di chuyển"""
        try:
            total_points = len(self.robot_path)
            print(f"Bắt đầu vẽ {total_points} điểm")
            
            # Lệnh về home trước khi bắt đầu
            self.send_command("HOME")
            self.send_command("PU")  # Nâng bút lên
            time.sleep(1)
            
            # Theo dõi chuyển động giữa các điểm
            prev_x, prev_y, prev_pen = 0, 0, 0  # Giả sử bắt đầu từ gốc toạ độ
            
            # Lặp qua từng điểm trong đường đi robot
            for i, (x, y, pen) in enumerate(self.robot_path):
                # Kiểm tra dừng
                if self.stop_drawing:
                    break
                
                # Tính toán góc tại điểm đích
                angles = self.inverse_kinematics(x, y)
                if not angles:
                    print(f"Bỏ qua điểm {i}: Ngoài tầm với ({x}, {y})")
                    continue
                
                theta1, theta2 = angles
                
                # Kiểm tra xem đây có phải là chuyển động nhấc bút và dời xa không
                is_long_move = False
                if prev_pen == 0 and pen == 0:  # Cả hai điểm đều có bút nhấc lên
                    distance = np.sqrt((x - prev_x)**2 + (y - prev_y)**2)
                    if distance > 20:  # Nếu khoảng cách đủ xa
                        is_long_move = True
                
                if is_long_move:
                    # Tạo animation cho chuyển động dài giữa các đoạn vẽ
                    self.animate_long_move(prev_x, prev_y, x, y, self.prev_angles, [theta1, theta2])
                else:
                    # Điều khiển robot thực tế
                    self.move_physical_robot(self.prev_angles, theta1, theta2, pen)
                    
                    # Hiển thị mô phỏng
                    self.root.after(0, lambda idx=i: self.simulate_robot_arm(self.robot_path, idx))
                
                # Cập nhật góc hiện tại
                self.prev_angles = [theta1, theta2]
                prev_x, prev_y, prev_pen = x, y, pen
                
                # Cập nhật tiến độ
                progress = (i + 1) / total_points * 100
                self.root.after(0, lambda p=progress: self.update_progress(p))
                
                # Chờ một chút giữa các điểm nếu pen_state = 1 (đang vẽ)
                if pen == 1:
                    time.sleep(self.step_size * 0.01)
                
            # Nâng bút khi kết thúc
            self.send_command("PU")
            
            # Về home sau khi vẽ
            self.send_command("HOME")
            
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Lỗi", f"Lỗi trong quá trình vẽ: {str(e)}"))
        finally:
            # Cập nhật trạng thái
            self.is_drawing = False
            self.root.after(0, self.reset_drawing_ui)

    def update_progress(self, progress):
        """Cập nhật thanh tiến độ"""
        self.progress_var.set(f"Tiến độ: {progress:.1f}%")
        self.progress['value'] = progress
    
    def reset_drawing_ui(self):
        """Reset giao diện sau khi vẽ xong"""
        self.draw_btn.config(state=tk.NORMAL)
        self.stop_btn.config(state=tk.DISABLED)
        
        if self.stop_drawing:
            messagebox.showinfo("Thông báo", "Quá trình vẽ đã bị dừng!")
        else:
            messagebox.showinfo("Thành công", "Vẽ hoàn thành!")
            self.progress_var.set("Tiến độ: 100%")
            self.progress['value'] = 100
    
    def stop_drawing_command(self):
        """Dừng quá trình vẽ"""
        if not self.is_drawing:
            return
            
        result = messagebox.askquestion("Xác nhận", "Bạn có chắc muốn dừng quá trình vẽ?")
        if result != 'yes':
            return
            
        self.stop_drawing = True
        
        # Dừng các động cơ
        self.send_command("PU")  # Nâng bút
    
    def emergency_stop(self):
        """Dừng khẩn cấp"""
        self.stop_drawing = True
        
        # Gửi lệnh dừng khẩn cấp
        self.send_command("STOP")
        
        messagebox.showwarning("Dừng khẩn cấp", "Lệnh dừng khẩn cấp đã được gửi!")
        
        # Reset UI
        self.reset_drawing_ui()
    
    def execute_gcode_process(self):
        """Thực thi G-code trên máy CNC thực tế"""
        try:
            if not self.gcode_list:
                messagebox.showinfo("Thông báo", "Không có G-code nào để thực thi.")
                return
                
            total_lines = len(self.gcode_list)
            print(f"Bắt đầu thực thi {total_lines} dòng G-code")
            
            # Thực thi từng dòng G-code
            for i, line in enumerate(self.gcode_list):
                # Kiểm tra dừng
                if self.stop_drawing:
                    break
                
                # Bỏ qua comment và dòng trống
                if line.strip().startswith(';') or not line.strip():
                    continue
                
                # Gửi G-code
                success = self.send_gcode(line)
                if not success:
                    print(f"Lỗi khi gửi dòng {i+1}: {line}")
                    continue
                
                # Cập nhật tiến độ
                progress = (i + 1) / total_lines * 100
                self.root.after(0, lambda p=progress: self.update_progress(p))
                
                # Chờ một chút để máy CNC thực hiện lệnh
                time.sleep(0.1)
            
            print("Thực thi G-code hoàn tất")
            
        except Exception as e:
            self.root.after(0, lambda: messagebox.showerror("Lỗi", f"Lỗi trong quá trình thực thi G-code: {str(e)}"))
        finally:
            # Cập nhật trạng thái
            self.is_drawing = False
            self.root.after(0, self.reset_drawing_ui)

    def animate_long_move(self, start_x, start_y, end_x, end_y, start_angles, end_angles):
        """Tạo animation cho chuyển động dài giữa các đoạn vẽ"""
        # Số lượng bước cho animation
        num_steps = 20
        
        # Tính toán bước dịch chuyển
        dx = (end_x - start_x) / num_steps
        dy = (end_y - start_y) / num_steps
        
        # Tính toán bước thay đổi góc
        d_theta1 = (end_angles[0] - start_angles[0]) / num_steps
        d_theta2 = (end_angles[1] - start_angles[1]) / num_steps
        
        # Thực hiện animation
        for step in range(num_steps + 1):
            # Tính toạ độ hiện tại
            current_x = start_x + dx * step
            current_y = start_y + dy * step
            
            # Tính góc hiện tại
            current_theta1 = start_angles[0] + d_theta1 * step
            current_theta2 = start_angles[1] + d_theta2 * step
            
            # Tạo điểm tạm thời để mô phỏng
            temp_point = (current_x, current_y, 0)  # Bút luôn nhấc lên trong chuyển động dài
            
            # Hiển thị mô phỏng với điểm tạm thời
            self.root.after(0, lambda pt=temp_point, t1=current_theta1, t2=current_theta2: 
                            self.simulate_arm_at_point(pt, t1, t2))
            
            # Điều khiển robot thực tế di chuyển đến vị trí trung gian
            if step % 4 == 0:  # Chỉ gửi lệnh sau mỗi 4 bước để tránh quá tải
                self.move_physical_robot_smooth(current_theta1, current_theta2, 0)
            
            # Chờ một khoảng thời gian ngắn
            time.sleep(0.05)  # Tốc độ animation - càng thấp càng nhanh

    def simulate_arm_at_point(self, point, theta1, theta2):
        """Mô phỏng cánh tay robot tại một điểm cụ thể"""
        self.ax_robot.clear()
        
        x, y, pen = point
        
        # Vẽ lưới nhẹ làm nền
        self.ax_robot.grid(True, linestyle='--', alpha=0.3)
        
        # Nếu đang vẽ thực tế, hiển thị đường đã vẽ
        if hasattr(self, 'drawn_path_x') and hasattr(self, 'drawn_path_y'):
            self.ax_robot.plot(self.drawn_path_x, self.drawn_path_y, 'g-', linewidth=1.5, zorder=2)
        
        # Tính toán vị trí điểm nối và điểm cuối
        x1 = self.L1 * np.cos(np.radians(theta1))
        y1 = self.L1 * np.sin(np.radians(theta1))
        
        x2 = x1 + self.L2 * np.cos(np.radians(theta1 + theta2))
        y2 = y1 + self.L2 * np.sin(np.radians(theta1 + theta2))
        
        # Vẽ cánh tay robot với độ dày và màu sắc rõ ràng hơn
        self.ax_robot.plot([0, x1], [0, y1], 'ro-', linewidth=4, markersize=8, zorder=3)  # Link 1
        self.ax_robot.plot([x1, x2], [y1, y2], 'bo-', linewidth=4, markersize=8, zorder=3)  # Link 2
        
        # Hiển thị đầu bút với màu sắc khác nhau tùy trạng thái
        pen_color = 'red' if pen == 0 else 'green'
        self.ax_robot.scatter([x2], [y2], s=100, color=pen_color, edgecolor='black', zorder=4)
        
        # Hiển thị trạng thái bút
        pen_status = "Đang vẽ" if pen == 1 else "Nhấc lên"
        self.ax_robot.set_title(f"Mô phỏng robot - Di chuyển... - Bút: {pen_status}")
        
        # Thiết lập các thuộc tính khác của đồ thị
        # Vẽ vùng làm việc
        circle = plt.Circle((0, 0), self.L1 + self.L2, fill=False, color='gray', linestyle='--', alpha=0.5)
        self.ax_robot.add_patch(circle)
        
        if abs(self.L1 - self.L2) > 1:
            inner_circle = plt.Circle((0, 0), abs(self.L1 - self.L2), fill=False, color='gray', linestyle='--', alpha=0.5)
            self.ax_robot.add_patch(inner_circle)
        
        # Vẽ trục tọa độ
        self.ax_robot.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        self.ax_robot.axvline(x=0, color='k', linestyle='-', alpha=0.3)
        
        # Thiết lập giới hạn trục
        limit = self.L1 + self.L2 + 50
        self.ax_robot.set_xlim(-limit, limit)
        self.ax_robot.set_ylim(-limit, limit)
        self.ax_robot.set_aspect('equal')
        
        # Cập nhật canvas
        self.canvas_robot.draw()

    def move_physical_robot_smooth(self, theta1, theta2, pen):
        """Di chuyển robot thực tế đến một vị trí cụ thể mà không cần chia nhỏ chuyển động"""
        if not self.is_connected or not self.arduino:
            return False

        try:
            # Tính số bước cho động cơ dựa trên góc
            dx = int(theta1 * self.step_per_mm)
            dy = int(theta2 * self.step_per_mm)

            # Gửi lệnh quay tuyệt đối
            self.send_command(f"GOTO {dx} {dy}")

            # Gửi lệnh điều khiển bút nếu cần
            if pen == 1 and (not hasattr(self, 'current_pen') or self.current_pen != 1):
                self.send_command("PD") 
                time.sleep(self.servo_delay)  # Đợi servo hoàn thành
                self.current_pen = 1
            elif pen == 0 and (not hasattr(self, 'current_pen') or self.current_pen != 0):
                self.send_command("PU")  
                time.sleep(self.servo_delay)  # Đợi servo hoàn thành
                self.current_pen = 0

            return True
        except Exception as e:
            print(f"Lỗi di chuyển robot: {str(e)}")
            return False
        
# Chạy ứng dụng
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotArmController(root)
    root.mainloop()