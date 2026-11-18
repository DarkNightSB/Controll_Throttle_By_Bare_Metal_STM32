# Điều khiển thử bướm ga điện tử Toyota bằng STM32F103

Điều khiển vị trí bướm ga điện tử Toyota **22030-0H030** (động cơ 2AZ-FE, cảm biến Hall hai track) trên bàn thử. Bàn đạp ga được mô phỏng bằng biến trở, mô-tơ điều khiển qua module **BTS7960**, kín vòng theo phản hồi VTA1/VTA2. Firmware lập trình **thanh ghi (CMSIS), không dùng HAL**.

> ⚠️ **Chỉ dùng thử trên bàn, bướm ga tách khỏi động cơ. Chưa dùng cho xe thật.**

## Mục lục

- [Phần cứng](#phần-cứng)
- [Sơ đồ đấu dây](#sơ-đồ-đấu-dây)
- [Nguyên lý hoạt động](#nguyên-lý-hoạt-động)
- [Cơ chế an toàn](#cơ-chế-an-toàn)
- [Hiệu chuẩn](#hiệu-chuẩn)
- [Log UART](#log-uart)
- [Chạy thử](#chạy-thử)
- [Xử lý sự cố](#xử-lý-sự-cố)
- [Lưu ý](#lưu-ý)

## Phần cứng

| Linh kiện | Ghi chú |
|---|---|
| STM32F103C8T6 (Blue Pill) | HSE 8 MHz, SYSCLK 72 MHz |
| Bướm ga Toyota 22030-0H030 | Mô-tơ DC khoảng 23 đến 38 Ω; cảm biến Hall VTA/VTA2, cấp 5 V |
| Module BTS7960 | Nguồn mô-tơ 12 V, logic 3,3 V |
| Nguồn 12 V | Nên có giới hạn dòng 0,5 đến 1 A khi thử |
| Biến trở bàn đạp | Hai đầu ngoài nối 3V3 và GND |
| USB-TTL | Xem log UART, gửi lệnh dừng |

## Sơ đồ đấu dây

Tất cả GND nối chung một điểm (nguồn 12 V, BTS7960, Blue Pill, `E2`, USB-TTL).

| Khối | Từ | Đến |
|---|---|---|
| Bàn đạp | Chân giữa biến trở | **PA1** |
| VTA1 | Dây `VTA` | 20 kΩ → **PA3**; PA3 → 10 kΩ → GND; PA3 → 100 nF → GND |
| VTA2 | Dây `VTA2` | 20 kΩ → **PB0**; PB0 → 10 kΩ → GND; PB0 → 100 nF → GND |
| Cảm biến | `VC` / `E2` | 5V Blue Pill / GND |
| BTS7960 | `RPWM` (đóng) | **PB8** (TIM4_CH3) |
| BTS7960 | `LPWM` (mở) | **PB9** (TIM4_CH4) |
| BTS7960 | `R_EN` / `L_EN` | **PA8** / **PA11**, mỗi chân kèm 10 kΩ xuống GND |
| BTS7960 | `VCC` | 3V3 |
| BTS7960 | `B+` / `B−` | 12 V (+) / (−), thêm tụ 470 đến 1000 µF sát module |
| BTS7960 | `M+`, `M−` | Hai dây mô-tơ bướm ga |
| UART | PA9 (TX), PA10 (RX) | RX, TX của USB-TTL (115200, 8N1) |

`R_IS`, `L_IS` để trống. Cầu chia áp VTA hệ số 1/3 để điện áp ở chân ADC luôn dưới 3,3 V.

## Nguyên lý hoạt động

- **Đọc ADC:** ADC1 quét PA1, PA3, PB0 bằng DMA circular, trung bình trượt 20 mẫu.
- **Mục tiêu:** bàn đạp chia 10 mức, mỗi mức tương ứng một phần hành trình.
- **Phản hồi:** vị trí là trung bình hai track VTA1/VTA2, đã chuẩn hóa theo mốc nghỉ và mốc mở.
- **PID** chạy mỗi 20 ms: Kp 80, Ki 2, Kd 1, giới hạn tích phân 30, vùng chết 2 %.
- **PWM:** TIM4 20 kHz; duty tối thiểu 10 %, tối đa 60 %. Chỉ một chân PWM hoạt động tại một thời điểm, dead-time 2 ms khi đảo chiều.
- **Chiều quay (đã xác nhận trên bàn):** `LPWM` là mở, `RPWM` là đóng.

## Cơ chế an toàn

1. Cầu H tắt ngay khi khởi động.
2. `armed`: chỉ cho chạy sau khi bàn đạp nhả (raw dưới 80) lúc khởi động.
3. Cửa sổ cảm biến: VTA1/VTA2 ngoài dải raw cho phép thì tắt cầu H, chờ bàn đạp nhả mới chạy lại.
4. Gửi `S` qua UART: dừng khẩn cấp, chỉ reset mới thoát.
5. Chặn lệnh mở khi một track chạm mốc `FULL_RAW`; chặn lệnh đóng khi cả hai track về gần mốc nghỉ.
6. Chờ đủ 20 mẫu ADC trước khi điều khiển.

## Hiệu chuẩn

Các hằng số nằm ở đầu `main.c`:

| Hằng số | Ý nghĩa |
|---|---|
| `VTA1_IDLE_RAW`, `VTA2_IDLE_RAW` | Raw ở vị trí nghỉ (lò xo limp-home) |
| `VTA1_FULL_RAW`, `VTA2_FULL_RAW` | Mốc "mở hết" dùng để chuẩn hóa và chặn mở |
| `VTA*_MIN_RAW`, `VTA*_MAX_RAW` | Cửa sổ phát hiện lỗi cảm biến |
| `MIN_DUTY_PERMILLE`, `MAX_DUTY_PERMILLE` | Giới hạn duty |

**Cách lấy mốc `FULL` thật:**

1. Tắt nguồn 12 V.
2. Dùng tay đẩy nhẹ cánh bướm tới cữ mở.
3. Đọc `VTA1_RAW` và `VTA2_RAW` trong log.
4. Đặt mốc `FULL` thấp hơn khoảng 20 raw.

Nếu cữ thật vượt `VTA*_MAX_RAW` thì nới cận này, nếu không sẽ báo lỗi cảm biến.

## Log UART

In mỗi 200 ms, ở mọi trạng thái (115200, 8N1):

```text
PEDAL_RAW,VTA1_RAW,VTA2_RAW,MOTOR_ACTIVE,DUTY_PERMILLE,
R_EN_ODR,L_EN_ODR,CCR3,CCR4,ARMED,SENSOR_FAULT,FAULT_WAITING,FAULT_LATCHED
```

## Chạy thử

1. Kiểm tra thông mạch GND chung; `VC` không được thông với 12 V hay `M±`.
2. Cấp USB trước, 12 V sau. Lúc cấp điện bàn đạp phải ở nhỏ nhất (PA1 dưới khoảng 0,07 V).
3. Vặn bàn đạp tăng dần, theo dõi log.

## Xử lý sự cố

| Hiện tượng | Kiểm tra |
|---|---|
| `R_EN`, `L_EN`, `RPWM`, `LPWM` đều 0 V | `ARMED`, `SENSOR_FAULT`, `FAULT_WAITING`, `FAULT_LATCHED` trong log |
| `ARMED=0` | Bàn đạp raw còn từ 80 trở lên lúc khởi động |
| `SENSOR_FAULT=1` | Đo PA3 (khoảng 0,24 đến 1,53 V) và PB0 (khoảng 0,64 đến 2,0 V) ở vị trí nghỉ |
| Bướm ga không mở hết | `DUTY` bằng 0 và VTA gần `FULL_RAW`: nâng mốc `FULL`. `DUTY` cao mà VTA không tăng: đã tới cữ cơ khí |
| Quay ngược chiều | Đổi `M+` với `M−` |

## Lưu ý

- Mô-tơ kẹt ở 12 V chỉ khoảng 0,5 A, nhưng vẫn không để đẩy vào cữ lâu.
- Bàn đạp thật trên xe là cảm biến riêng; bản này chỉ mô phỏng bằng biến trở.
- Quạt (PA6), nút nhấn (PA0), biến trở nhiệt (PA2) và LCD I2C (PB6/PB7) thuộc bản `main.c` đầy đủ, không có trong bản test này.
