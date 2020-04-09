class AX_RX_XL
{

    byte  packet[30];
    unsigned short crc;
   
    public:
    AX_RX_XL(unsigned char id)
    void led_XL(char r, char g, char b);
    void joint_XL(unsigned int position, unsigned int speed, unsigned int time);
    void wheel_XL(unsigned int speed, unsigned char cw_ccw);
    void goto_joint_mode_XL();
    void goto_wheel_mode_XL();
    void angle_limit_XL(unsigned int cw_limit, unsigned int ccw_limit);
    void speed_XL(unsigned int speed);
    void torque_XL(unsigned char State);
    void change_id_XL(unsigned char new_id);
    void change_baudrate_XL(unsigned char baudrate_code);
    void return_level_XL(char level);
  private:
    unsigned short update_crc_XL(unsigned short crc_accum, unsigned char *data_blk_ptr, unsigned short data_blk_size);
};

