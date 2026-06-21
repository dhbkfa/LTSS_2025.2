# Báo cáo Tổng quan Dự án: Mô phỏng Đa Robot Khám phá Bản đồ (LTSS_2025.2)

## 1. Giới thiệu tổng quan đề tài
Dự án **"Mô phỏng 5 robot song song khám phá bản đồ lưới chưa biết"** được xây dựng nhằm giải quyết bài toán tìm kiếm và khám phá không gian chưa biết trong môi trường thực tế (như tìm kiếm cứu nạn, lập bản đồ tự động). 
Hệ thống kết hợp các thuật toán hiện đại bao gồm **Frontier-Based Exploration** (khám phá dựa trên ranh giới), **A* Search** (tìm đường đi ngắn nhất) và **Reservation Table** (bảng đặt chỗ để tránh va chạm). Toàn bộ logic lõi được lập trình bằng C++ đa luồng (multi-threading) nhằm tối ưu hiệu năng tính toán, trong khi giao diện (UI) được xây dựng bằng Python để trực quan hóa toàn bộ quá trình mô phỏng.

## 2. Mô hình hóa bài toán (Problem Modeling)
* **Môi trường (Environment):** Bản đồ lưới 2D rời rạc (Grid Map). Trạng thái của một ô lưới có thể là: Chưa biết (UNKNOWN), Trống (FREE), Vật cản (OCCUPIED), Vùng cấm do giãn nở (INFLATED_OBSTACLE), Đã thăm (VISITED), hoặc Đích (TARGET).
* **Tác nhân (Agents):** Nhóm gồm 5 robot. Mỗi robot có kích thước (bán kính) và khả năng cảm biến quanh mình.
* **Biết trước:** Kích thước một ô lưới, vị trí xuất phát, thuật toán điều khiển.
* **Không biết trước:** Kích thước tổng thể của bản đồ, hình dáng vật cản, vị trí đích thực sự (hoặc liệu có đích hay không).
* **Điều kiện dừng (Mode 3 - Kết hợp):** Nếu có đích thật, dừng lại ngay khi có một robot đạp trúng đích. Nếu không có đích thật, mô phỏng sẽ dừng lại khi toàn bộ bản đồ có thể đi được đã được phủ kín (không còn ô ranh giới).

## 3. Hàm mục tiêu (Optimization / Objective Function)
Dự án được tối ưu hóa (Optimize) theo các mục tiêu chính sau:
1. **Tối ưu hóa thời gian (Minimizing Makespan):** Tối thiểu hóa tổng số bước thời gian (time-steps) cần thiết để tìm thấy mục tiêu hoặc phủ kín bản đồ. Điều này đạt được thông qua việc phân công vùng tìm kiếm hợp lý (Task Allocation) sao cho 5 robot toả ra các hướng khác nhau, tránh việc các robot tập trung dẫm chân lên cùng một khu vực.
2. **Tối ưu hóa chi phí đường đi (Minimizing Path Cost):** Giảm thiểu quãng đường di chuyển của từng robot đến các điểm cần khám phá (Frontiers). Thuật toán A* được sử dụng để đảm bảo đường đi tìm được là ngắn nhất từ vị trí hiện tại đến đích ảo.
3. **An toàn tuyệt đối (Safety / Zero Collision):** Đảm bảo không có bất kỳ va chạm nào xảy ra giữa các robot hoặc giữa robot với tường (được đảm bảo bởi Reservation Table và cơ chế Inflated Obstacle - giãn nở vật cản).

## 4. Các Model / Thuật toán sử dụng
Hệ thống sử dụng sự kết hợp của 3 mô hình cốt lõi:
* **Mô hình Khám phá (Frontier-Based Exploration):** Hệ thống liên tục quét tìm các "Frontier" (biên) - là những ô lưới đã biết là trống (FREE) hoặc đã thăm (VISITED) nhưng nằm kề cạnh với các vùng không xác định (UNKNOWN). Frontier đóng vai trò là "đích ảo" để robot hướng tới.
* **Mô hình Tìm đường (A* Heuristic Search):** Sau khi mỗi robot được giao một Frontier, thuật toán A* sẽ được gọi để tìm quỹ đạo di chuyển (tập hợp các ô lưới liên tiếp) ngắn nhất đến đó.
* **Mô hình Phối hợp và Tránh va chạm (Reservation Table / Lập lịch không gian - thời gian):** Để tránh va chạm, quỹ đạo của từng robot sẽ được đăng ký vào một "bảng đặt chỗ". Nếu phát hiện hai robot cùng chiếm một ô tại cùng một thời điểm (trùng ô) hoặc đi ngược chiều nhau trên cùng một cạnh lưới (đổi chỗ đối đầu), hệ thống sẽ từ chối đường đi của robot có độ ưu tiên thấp hơn và yêu cầu nó đứng chờ hoặc tìm đường khác.

## 5. Luồng thực thi, Multi-Core và Khả năng Tăng tốc (Tuần tự vs Song song)
* **Luồng thực thi hệ thống:**
  1. *Sense:* 5 robot dùng cảm biến cập nhật bản đồ chung (SharedMap).
  2. *Plan:* Tìm các Frontier và phân công cho từng robot. Các robot độc lập chạy A* để tìm đường.
  3. *Coordinate:* Hệ thống kiểm tra va chạm bằng Reservation Table.
  4. *Move:* Các robot hợp lệ sẽ tiến lên một bước. Cập nhật lại UI.
* **Tuần tự (Sequential):** Nếu chạy tuần tự, khi hệ thống tính đường cho robot 1, 4 robot kia phải đứng chờ. Thuật toán A* trên bản đồ lớn rất tốn CPU, việc tính toán lần lượt sẽ gây thắt cổ chai, làm giảm FPS của mô phỏng.
* **Song song (Parallel - Đa luồng/Multi-threading):** 
  * Dự án sử dụng `std::thread` của C++ để cấp cho mỗi robot một luồng (thread) hoạt động riêng biệt.
  * Việc nhận diện môi trường và chạy thuật toán A* của 5 robot được **thực thi hoàn toàn song song** trên các nhân CPU (Core) khác nhau. 
  * Chỉ có bước cập nhật bản đồ chung (SharedMap) và kiểm tra va chạm trong bảng đặt chỗ là cần đồng bộ hóa thông qua `std::mutex` bởi một tiến trình điều phối (Coordinator).
  * **Khả năng tăng tốc:** Việc áp dụng lập trình song song giúp tăng tốc độ mô phỏng đáng kể (tăng tốc theo hệ số nhân dựa trên số lượng robot/luồng), cho phép chạy mượt mà ngay cả trên bản đồ có kích thước lớn.

## 6. Kế hoạch triển khai Code và Kịch bản diễn ra
* **Kiến trúc phần mềm phân tầng:**
  * **Tầng Logic (C++):** Phụ trách toàn bộ tính toán nặng (Map, Robot State, A*, Threading). Chạy ngầm và xuất dữ liệu trạng thái ra file JSON sau mỗi bước.
  * **Tầng Giao diện (Python):** `map_builder.py` để vẽ bản đồ tạo dữ liệu đầu vào. `run_viewer.py` đọc các file JSON do C++ sinh ra và render lên màn hình.
* **Kịch bản vận hành:**
  1. Người dùng mở UI Python (Tab 1), vẽ các vật cản, chọn từ 1 đến 5 điểm xuất phát cho robot và đặt 1 đích ẩn. Xuất bản đồ thành file JSON.
  2. Chuyển sang Tab 2, nạp bản đồ JSON và bấm "Chạy mô phỏng". 
  3. Tiến trình C++ được gọi lên chạy ngầm, liên tục xả log trạng thái ra thư mục `run_output/`.
  4. Giao diện Python lập tức lấy dữ liệu và hiển thị lên UI, người dùng có thể điều chỉnh tốc độ, tua lùi/tới qua từng timestep để quan sát các robot dần dần làm sáng bản đồ và tìm thấy đích.

## 7. Nhận xét và Đánh giá (Đưa vào báo cáo)
* **Ưu điểm:**
  * Thiết kế kiến trúc rất tách bạch và module hóa (C++ xử lý thuật toán tối ưu, Python đảm nhận giao diện trực quan).
  * Áp dụng thành công các kiến thức cốt lõi của Lập trình song song. Giải quyết tốt bài toán chạy đa luồng an toàn bằng `std::mutex` và cơ chế tránh Deadlock/Livelock.
  * Reservation Table xử lý triệt để các tình huống khó như "đổi chỗ đối đầu", "nhiều robot cùng hướng về 1 điểm" mà không phá vỡ tính nhất quán.
* **Nhược điểm và Hướng phát triển:**
  * Thuật toán Reservation Table hiện đang kiểm tra chéo (O(n^2)), rất tốt cho nhóm 5-10 robot nhưng sẽ bị chậm nếu mở rộng lên hàng chục/hàng trăm robot (Swarm Robotics).
  * Giao tiếp giữa C++ và Python thông qua việc ghi/đọc hàng ngàn file JSON nhỏ đang là phương pháp đơn giản nhưng chưa tối ưu về mặt I/O ổ cứng. Trong tương lai có thể nâng cấp lên giao tiếp qua Socket (TCP/UDP) hoặc Shared Memory để đạt thời gian thực (Real-time) trên quy mô lớn hơn.
