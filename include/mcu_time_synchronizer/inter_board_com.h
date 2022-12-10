#ifndef INTER_BOARD_H_
#define INTER_BOARD_H_

#include "sys/select.h"
#include "libusb-1.0/libusb.h"
#include <ctime>
#include <iostream>
#include <pthread.h>
#include <exception>

namespace curi {

struct Vendor_id_Hex {
  explicit Vendor_id_Hex(uint16_t _vendor_id) : vendor_id(_vendor_id) {}
  uint16_t vendor_id;
};

struct Product_id_Hex {
  explicit Product_id_Hex(uint16_t _product_id_Hex)
      : product_id(_product_id_Hex) {}
  uint16_t product_id;
};

class USB_COM {
 public:
  explicit USB_COM(int in_length_8b,
                   int out_length_8b,
                   Vendor_id_Hex _vendor_id,
                   Product_id_Hex _product_id);
  virtual ~USB_COM();
  virtual void Deal_Out_Data() = 0;
  virtual void Deal_In_Data() = 0;
  virtual void USB_Com_Start_Trans_Asy(void(*cbf_wrapper)(struct libusb_transfer *)) = 0;
  virtual void USB_In_CBF(struct libusb_transfer *transfer) = 0;
  virtual void USB_Out_CBF(struct libusb_transfer *transfer) = 0;
  void lock_mutex_tx();
  void unlock_mutex_tx();
  void lock_mutex_rx();
  void unlock_mutex_rx();
  libusb_context *&get_usb_ctx();
  libusb_device_handle *&get_device_handle();
  uint16_t get_product_id() const { return Product_id.product_id; }
  //maybe private+function is much better
 protected:
  uint8_t *tx_buff;
  uint8_t *rx_buff;
  int usb_message_in_length;
  int usb_message_out_length;
  int usb_message_in_checklength;
  int usb_message_out_checklength;
  int usb_message_32_2_8;
  int usb_message_8_2_32;

  libusb_transfer *transfer_tx;
  libusb_transfer *transfer_rx;
  libusb_context *ctx;
 private:
  Vendor_id_Hex Vendor_id;
  Product_id_Hex Product_id;
  libusb_device_handle *deviceHandle;
  pthread_mutex_t usb_com_tx_mutex;
  pthread_mutex_t usb_com_rx_mutex;
};

} // namespace curi

#endif