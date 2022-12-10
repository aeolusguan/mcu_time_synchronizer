#ifndef USB_IMU_H_
#define USB_IMU_H_

#include "inter_board_com.h"
#include <cstring>

namespace curi {

typedef struct {
  int real_utc;
  int date_utc;
  double st_time;
  uint32_t checksum;
} usb_utc_rx_cmd_t;

typedef struct {
  int real_utc;
  int date_utc;
  double st_time;
} usb_utc_rx_data_t;

class USB_COM_UTC : public curi::USB_COM {
 public:
  USB_COM_UTC(int in_length_8b,
              int out_length_8b,
              Vendor_id_Hex _vendor_id,
              Product_id_Hex _product_id,
              unsigned char _endpoint_in,
              unsigned char _endpoit_out);

  ~USB_COM_UTC() override;

  void Deal_Out_Data() override;

  void Deal_In_Data() override;

  void USB_Com_Start_Trans_Asy(void(*cbf_wrapper)(struct libusb_transfer *)) override;

  void USB_In_CBF(struct libusb_transfer *transfer) override;

  void USB_Out_CBF(struct libusb_transfer *transfer) override;

  void Print_Rx_Data();

  void turn_on_print_data() { if_print = true; }

  inline const usb_utc_rx_data_t *get_usb_received_data() {
    return usb_data_drv;
  }

 private:
  uint64_t frame_id;
  usb_utc_rx_cmd_t *usb_in_data;
  usb_utc_rx_data_t *usb_data_drv;
  unsigned char endpoint_in;
  unsigned char endpoint_out;
  bool if_print;
};

} // namespace curi
#endif