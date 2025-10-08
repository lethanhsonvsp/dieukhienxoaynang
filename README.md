Hướng dẫn sử dụng code điều khiển cơ cấu xoay và nâng bằng giao thức CAN (SocketCAN) trên hệ thống Linux (Ubuntu). Code này sử dụng C# để giao tiếp với bus CAN, điều khiển hai động cơ (một cho xoay, một cho nâng) thông qua PDO (Process Data Object) và SDO (Service Data Object). Dưới đây là hướng dẫn chi tiết bằng tiếng Việt:

---

### **Tổng quan**
Code điều khiển một hệ thống cơ cấu xoay và nâng, sử dụng hai động cơ với Node ID là 1 (xoay) và 2 (nâng). Các động cơ được điều khiển thông qua giao thức CANopen, sử dụng SocketCAN để giao tiếp với bus CAN. Tốc độ của động cơ được điều chỉnh bằng phím mũi tên trên bàn phím, với các thông số như sau:
- **Node ID 1**: Điều khiển động cơ xoay, sử dụng tỉ số truyền (`gear_ratio_rotate_motor = 350`).
- **Node ID 2**: Điều khiển động cơ nâng, sử dụng hệ số chuyển đổi (`convert_lift_param = 33` rpm tương ứng 1 mm/s).
- **Điều khiển**: Sử dụng phím mũi tên (lên/xuống) để tăng/giảm tốc độ, phím "S" để dừng, phím "C" để thoát.

Code hỗ trợ:
- **Chế độ tốc độ (Velocity Mode)**: Điều khiển tốc độ quay của động cơ.
- **Chế độ vị trí (Position Mode)**: (Hiện bị comment, có thể bật nếu cần).
- **PDO**: Gửi và nhận dữ liệu thời gian thực (RPDO/TPDO).
- **SDO**: Cấu hình và kiểm tra trạng thái động cơ.

---

### **Yêu cầu phần cứng**
- **Giao diện CAN**: Thiết bị USB-to-CAN (ví dụ: PEAK-System, Kvaser) kết nối với bus CAN.
- **Hai động cơ**: Hỗ trợ chuẩn CANopen (CiA402), với Node ID 1 (xoay) và 2 (nâng).
- **Máy tính chạy Ubuntu**: Đã cài đặt môi trường .NET và các công cụ CAN.

---

### **Yêu cầu phần mềm**
1. **Hệ điều hành**: Ubuntu (khuyến nghị 20.04 hoặc mới hơn).
2. **.NET SDK**: Phiên bản tương thích (ví dụ: .NET 6 hoặc 8).
   - Cài đặt:
     ```bash
     sudo apt update
     sudo apt install dotnet-sdk-6.0
     ```
3. **can-utils**: Công cụ dòng lệnh để quản lý CAN.
   - Cài đặt:
     ```bash
     sudo apt install can-utils
     ```
4. **Thư viện SocketCAN**: Đã tích hợp trong Linux kernel, không cần cài thêm.
5. **Quyền root**: Một số lệnh CAN yêu cầu `sudo`.

---

### **Cài đặt và cấu hình**
1. **Kiểm tra giao diện CAN**:
   - Kết nối thiết bị CAN vào máy tính.
   - Chạy lệnh để kiểm tra giao diện:
     ```bash
     ip link show
     ```
     - Kết quả nên hiển thị giao diện `can0`.
   - Nếu không thấy, load module CAN:
     ```bash
     sudo modprobe can
     sudo modprobe can_raw
     ```

2. **Thiết lập giao diện CAN**:
   - Thiết lập bitrate (mặc định 500000 trong code trước, nhưng code này không cấu hình bitrate, cần thiết lập thủ công):
     ```bash
     sudo ip link set can0 type can bitrate 500000
     sudo ip link set can0 up
     ```
   - Kiểm tra trạng thái:
     ```bash
     ip -details link show can0
     ```
     - Đảm bảo giao diện ở trạng thái `UP`.

3. **Cài đặt dự án .NET**:
   - Tạo thư mục dự án và sao chép code vào file `Program.cs`.
   - Biên dịch và chạy:
     ```bash
     dotnet build
     sudo dotnet run
     ```

---

### **Cách sử dụng**
1. **Khởi động chương trình**:
   - Chạy lệnh:
     ```bash
     sudo dotnet run
     ```
   - Chương trình sẽ:
     - Mở socket CAN và bind với giao diện `can0`.
     - Gửi lệnh NMT (Network Management) để khởi động hai động cơ (Node 1 và 2).
     - Cấu hình PDO mặc định cho cả hai động cơ.
     - Kích hoạt chế độ Velocity Mode (hoặc Position Mode nếu bỏ comment).
     - Bắt đầu luồng đọc phím và lắng nghe TPDO.

2. **Điều khiển động cơ**:
   - **Phím mũi tên lên**: Đặt tốc độ = 10 (tương ứng 10 * 350 rpm cho xoay, 10 * 33 rpm cho nâng).
   - **Phím mũi tên xuống**: Đặt tốc độ = -10 (hướng ngược lại).
   - **Phím S**: Dừng cả hai động cơ (tốc độ = 0).
   - **Phím C**: Thoát chương trình, dừng động cơ.
   - **Ctrl+C hoặc thoát process**: Tự động gửi tốc độ = 0 để đảm bảo an toàn.

3. **Theo dõi trạng thái**:
   - Console hiển thị:
     - **RPDO1**: Statusword (trạng thái động cơ, ví dụ: 0x0027 = Operation Enabled).
     - **RPDO2**: Vị trí thực tế (Position Actual).
     - **RPDO3**: Tốc độ thực tế (Velocity Actual).
     - **RPDO4**: Cả vị trí và tốc độ thực tế.
     - **TPDO3**: Tốc độ mục tiêu gửi đi.
   - Ví dụ output:
     ```
     SocketCAN opened & bound to can0 (fd=3)
     Sent NMT Start to node 1
     Configuring PDO mapping...
     PDO mapping configured to default.
     [RPDO1] Statusword = 0x0027
     [RPDO3] Velocity actual = 10
     ```

4. **Dừng chương trình**:
   - Nhấn phím **C** để thoát an toàn (động cơ được đặt về tốc độ 0).
   - Hoặc nhấn **Ctrl+C**, chương trình sẽ tự động gửi tốc độ = 0 trước khi thoát.

---

### **Cấu hình nâng cao**
1. **Thay đổi giao diện CAN**:
   - Mặc định: `can0`.
   - Sửa trong `StartCanOpen`:
     ```csharp
     ifr_name = Encoding.ASCII.GetBytes("can1\0".PadRight(16, '\0'))
     ```

2. **Thay đổi Node ID**:
   - Mặc định: Node 1 (xoay), Node 2 (nâng).
   - Sửa trong `Program`:
     ```csharp
     static readonly uint nodeId = 3; // Node ID mới cho xoay
     static readonly uint nodeId2 = 4; // Node ID mới cho nâng
     ```

3. **Điều chỉnh tỉ số truyền và hệ số nâng**:
   - **Xoay**: `gear_ratio_rotate_motor = 350`.
   - **Nâng**: `convert_lift_param = 33` (1 mm/s = 33 rpm).
   - Sửa trong `Program`:
     ```csharp
     static readonly int gear_ratio_rotate_motor = 400; // Tỉ số truyền mới
     static readonly int convert_lift_param = 50; // Hệ số nâng mới
     ```

4. **Chuyển sang Position Mode**:
   - Mặc định sử dụng Velocity Mode. Để dùng Position Mode:
     - Bỏ comment các dòng trong `Main`:
       ```csharp
       ControlPDO.EnableMotorPositionMode(sock, nodeId);
       ControlPDO.EnableMotorPositionMode(sock, nodeId2);
       ControlPDO.SendPosition(sock, nodeId, 90000000, speed_rotation);
       ControlPDO.SendPosition(sock, nodeId2, 90000000, speed_lift);
       ```
     - Comment các dòng Velocity Mode:
       ```csharp
       // ControlPDO.EnableMotorVelocityMode(sock, nodeId);
       // ControlPDO.EnableMotorVelocityMode(sock, nodeId2);
       // ControlPDO.SendVelocity(sock, (uint)nodeId, speed_rotation);
       // ControlPDO.SendVelocity(sock, (uint)nodeId2, speed_lift);
       ```
     - Điều chỉnh `targetPos` (90000000) và `profileVelocity` trong `SendPosition` nếu cần.

5. **Tần suất gửi lệnh**:
   - Mặc định: Gửi tốc độ mỗi 1 giây (`Thread.Sleep(1000)`).
   - Sửa trong `Main` để tăng tốc độ cập nhật:
     ```csharp
     Thread.Sleep(500); // Gửi mỗi 0.5 giây
     ```

---

### **Khắc phục sự cố**
1. **Không mở được socket CAN**:
   - Kiểm tra giao diện: `ip link show`.
   - Đảm bảo can-utils được cài:
     ```bash
     sudo apt install can-utils
     ```
   - Thiết lập lại giao diện:
     ```bash
     sudo ip link set can0 down
     sudo ip link set can0 type can bitrate 500000
     sudo ip link set can0 up
     ```

2. **Động cơ không phản hồi**:
   - Kiểm tra Node ID có khớp với động cơ (1 và 2).
   - Xem Statusword trong console:
     - `0x0027`: Operation Enabled (bình thường).
     - `0x0008`: Fault (cần reset lỗi qua SDO).
   - Kiểm tra kết nối vật lý của bus CAN.
   - Đảm bảo động cơ hỗ trợ Velocity Mode (0x03) hoặc Position Mode (0x01).

3. **Lỗi gửi frame**:
   - Kiểm tra bitrate của CAN interface có khớp với động cơ (thường là 500000).
   - Dùng `candump` để debug:
     ```bash
     candump can0
     ```

4. **Chương trình không nhận phím**:
   - Đảm bảo console đang focus.
   - Tăng thời gian sleep trong `ReadKeys` nếu CPU quá tải:
     ```csharp
     Thread.Sleep(200); // Tăng từ 100ms lên 200ms
     ```

---

### **Lưu ý an toàn**
- **Dừng an toàn**: Nhấn phím **C** hoặc **Ctrl+C** để đảm bảo động cơ được đặt về tốc độ 0 trước khi thoát.
- **Kiểm tra trạng thái động cơ**: Đảm bảo Statusword = 0x0027 (Operation Enabled) trước khi gửi lệnh tốc độ/vị trí.
- **Quyền root**: Chạy với `sudo` để tránh lỗi truy cập CAN interface.
- **Bitrate CAN**: Đảm bảo bitrate của CAN interface khớp với động cơ.

---

### **Mở rộng**
1. **Thêm giao diện điều khiển**: Thay `ControlSpeed` bằng giao diện USB (như tay cầm PS5, tham khảo code trước) hoặc GUI.
2. **Điều khiển PID**: Dựa trên `Velocity actual` từ TPDO3 để thêm vòng điều khiển PID.
3. **Ghi log dữ liệu**: Lưu TPDO (vị trí, tốc độ) vào file để phân tích:
   ```csharp
   File.AppendAllText("log.txt", $"[RPDO3] Velocity actual = {vel}\n");
   ```
4. **Homing Mode**: Bật chế độ Homing bằng cách gọi `EnableMotorHomeMode` và cấu hình thêm.

---

### **Ví dụ kết quả console**
Khi chạy thành công:
```
SocketCAN opened & bound to can0 (fd=3)
Sent NMT Start to node 1
Configuring PDO mapping...
PDO mapping configured to default.
[RPDO1] Statusword = 0x0027
[TPDO3] Velocity target = 3500
[RPDO3] Velocity actual = 10
[RPDO4] Position actual = 123456
[RPDO4] Velocity actual = 3500
```

---

Nếu cần hỗ trợ thêm (debug lỗi, thêm tính năng, hoặc tích hợp với code trước), hãy cho tôi biết chi tiết!
