#include "mcu_time_synchronizer/usb_com.h"

curi::USB_COM_UTC::USB_COM_UTC(int in_length_8b,
                               int out_length_8b,
                               curi::Vendor_id_Hex _vendor_id,
                               curi::Product_id_Hex _product_id,
                               unsigned char _endpoint_in,
                               unsigned char _endpoit_out) :
    curi::USB_COM(in_length_8b,
                  out_length_8b,
                  _vendor_id,
                  _product_id),
    frame_id(0),
    endpoint_in(_endpoint_in),
    endpoint_out(_endpoit_out),
    if_print(false) {
  usb_in_data = new usb_utc_rx_cmd_t();
  usb_data_drv = new usb_utc_rx_data_t();
}

curi::USB_COM_UTC::~USB_COM_UTC() {
  delete usb_in_data;
  delete usb_data_drv;
}

void curi::USB_COM_UTC::USB_Com_Start_Trans_Asy(void (*cbf_wrapper)(
    struct libusb_transfer *)) {
  libusb_fill_interrupt_transfer(transfer_rx,
                                 get_device_handle(),
                                 endpoint_in,
                                 rx_buff,
                                 usb_message_in_length,
                                 cbf_wrapper,
                                 this,
                                 0);
  libusb_submit_transfer(transfer_rx);
  if (transfer_rx->status == 0) {
    std::cerr << "[USB IMU Asynchronously Receiving]\n";
  } else {
    std::cerr << "[Error] Can not start transmitting\n";
    std::abort();
  }
}

void curi::USB_COM_UTC::Deal_Out_Data() {

}

void curi::USB_COM_UTC::Deal_In_Data() {
  for (int i = 0; i < usb_message_8_2_32; i++) {
    ((uint32_t *) usb_in_data)[i] =
        (rx_buff[4 * i + 3] << 24) + (rx_buff[4 * i + 2] << 16)
            + (rx_buff[4 * i + 1] << 8) + rx_buff[4 * i];
  }

  auto *temp_cmd = (uint32_t *) usb_in_data;
  uint32_t t = 0;
  for (size_t i = 0; i < usb_message_in_checklength; i++)
    t = t ^ temp_cmd[i];
  if (usb_in_data->checksum == t) {
    memcpy(usb_data_drv, usb_in_data, usb_message_in_length - 4);
  } else {
    std::cerr << "[ERROR] USB RX CMD CHECKSUM ERROR!\n";
  }
}

void curi::USB_COM_UTC::USB_In_CBF(struct libusb_transfer *transfer) {
  if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
    std::cout << "[ERROR] Asy Trans Failed! Try again!\n";
    libusb_submit_transfer(transfer);
  } else if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
    frame_id++;
    lock_mutex_rx();
    this->Deal_In_Data();
    unlock_mutex_rx();
    if (if_print)
      this->Print_Rx_Data();
    libusb_submit_transfer(transfer);
  }
}

void curi::USB_COM_UTC::USB_Out_CBF(struct libusb_transfer *transfer) {

}

void curi::USB_COM_UTC::Print_Rx_Data() {
  std::cerr << "[Frame " << frame_id << " ] [Device " << get_product_id()
            << " UTC DATA] " << usb_data_drv->real_utc << " [Date] "
            << usb_data_drv->date_utc <<
            "[ST TIME] " << usb_data_drv->st_time << std::endl;
}