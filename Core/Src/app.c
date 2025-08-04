//#include "app.h"
//
//typedef enum
//{
//	DUNG,
//	TANG_TOC,
//	CHAY_THANG,
//	GIAM_TOC,
//	KHOI_DONG
//}NAME;
//
//typedef struct
//{
//	NAME name;
//	//parameter of each state
//
//}STATE;
//STATE state;
//void dung()
//{
//	state.name = DUNG;
//	//chuyen khoi dong
//}
//void tang_toc()
//{
//	state.name = TANG_TOC;
//	//ham xu ly
//
//}
//void chay_thang()
//{
//	state.name = CHAY_THANG;
//	//ham xu ly
//}
//void giam_toc()
//{
//	state.name = GIAM_TOC;
//	//ham xu ly
//}
//void khoi_dong̣()
//{
//	//state 1
//	// mo van duoi(g1 g2) nếu 3 giá trị xấp xỉ 12v thì lấy giá trị kp
//	PWM_Init();
//	// Hàm kiểm tra 3 giá trị có xấp xỉ 12V không
//	bool kiem_tra_ba_gia_tri_co_bang_12v_khong(float vout1, float vout2, float vout3) {
//	    // Kiểm tra từng giá trị trong khoảng 11.9V - 12.1V
//	    if (vout1 < 11.9f || vout1 > 12.1f) return false;
//	    if (vout2 < 11.9f || vout2 > 12.1f) return false;
//	    if (vout3 < 11.9f || vout3 > 12.1f) return false;
//	    return true; // Cả 3 giá trị đều xấp xỉ 12V
//	}
//	float vout_values[3] = {0.0f, 0.0f, 0.0f}; // Lưu 3 giá trị vout gần nhất
//	int index = 0; // Vị trí trong mảng
//	bool done = false; // Cờ để thoát vòng lặp
//	while (!done)
//	{
//	  // Lưu giá trị vào mảng (theo kiểu vòng: 0 -> 1 -> 2 -> 0 -> ...)
//	  vout_values[index] = adc_get_vout();
//	  index = (index + 1) % 3; // Tăng chỉ số, quay lại 0 nếu đến 3
//	  // Kiểm tra nếu đã có ít nhất 3 giá trị
//	  if (index == 0)
//	  {
//	   // Đã lấy đủ 3 giá trị (sau lần lặp thứ 3)
//	   // Kiểm tra 3 giá trị gần nhất
//	   if (kiem_tra_ba_gia_tri_co_bang_12v_khong(vout_values[0], vout_values[1], vout_values[2]))
//	   {
//	     done = true; // Thoát vòng lặp nếu xấp xỉ 12V
//	   }
//	  }
//	        // Chờ 1ms trước khi lấy mẫu tiếp theo
//	   HAL_Delay(1);
//	 }
//	//tính kp
//	float kp = 12/(vref-vc);
//
//	//state2
//	// e = vsp-x => ki
//}
//void excute()
//{
//	switch(state.name)
//	{
//		case DUNG:
//			//do something
//		break;
//		case GIAM_TOC:
//			//do something
//		break;
//		case TANG_TOC:
//			//do something
//		break;
//		case CHAY_THANG:
//			//do something
//		break;
//		default:
//			// do nothing
//		break;
//	}
//}
//void handle()
//{
//	excute();
//}
//void app_init()
//{
//	state.name = DUNG;
//	//set thong mach va bo pi
//	PI_IBM_2P_Start_wrapper();
//}
