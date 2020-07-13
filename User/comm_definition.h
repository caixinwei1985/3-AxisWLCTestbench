#ifndef COMM_DEFINITION_
#define COMM_DEFINITION_

#define REG_FAN1_W      0x0100
#define REG_FAN2_W      0x0101
#define REG_SW5V_W      0x0102
#define REG_SW24V_W     0X0103
#define REG_LAMP_W      0x0104
#define REG_LED_W       0x0105
#define REG_LASER_W     0x0106
#define REG_BUZZER_W    0x0107
#define REG_XDIR_W      0x0108
#define REG_YDIR_W      0x0109
#define REG_ZDIR_W      0x010A
#define REG_DRVEN_W     0x010B
#define REG_XPUL_W      0x010C
#define REG_YPUL_W      0x010D
#define REG_ZPUL_W      0x010E
#define REG_HPO1_W      0x010F
#define REG_HPO2_W      0x0110

#define REG_PROT0_R     0x0200
#define REG_START_R     0x0201
#define REG_JS1BT_R     0x0202
#define REG_JS1D0_R     0x0203
#define REG_JS1D1_R     0x0204
#define REG_JS1D2_R     0x0205
#define REG_JS1D3_R     0x0206
#define REG_JS2BT_R     0x0207
#define REG_JS2D0_R     0x0208
#define REG_JS2D1_R     0x0209
#define REG_JS2D2_R     0x020A
#define REG_JS2D3_R     0x020B
#define REG_XSW0_R      0x020C
#define REG_XSW1_R      0x020D
#define REG_XSW2_R      0x020E
#define REG_YSW0_R      0x020F
#define REG_YSW1_R      0x0210
#define REG_YSW2_R      0x0211
#define REG_ZSW0_R      0x0212
#define REG_ZSW1_R      0x0213
#define REG_ZSW2_R      0x0214
#define REG_NTC1_R      0x0215
#define REG_NTC2_R      0x0216
#define REG_NTC3_R      0x0217
#define REG_MOTCUR_R    0x0218
#define REG_PROT1_R     0x0219
#define REG_PROT2_R     0x021a
#define REG_VBAT_R      0x021b

#define CMD_MOTO_MOVE   0x0300  /* DATA 10 Bytes: byte0 axis,byte1 acc, byte2:byte3 speed,byte4:byte7 step,byte8:byte9 timeout */
#define CMD_MOTO_RST    0x0301  /* DATA  3 Byte:  byte0 axis,byte1:byte2 timeout  */

#define NTF_MOTO_COMP   0x0400
#define NTF_MOTO_OT     0x0401
#define NTF_PROT_TRIG   0x0402
#define NTF_MOTO_RESET  0x0403
#define NTF_MOTO_FAR    0x0404
#define NTF_MOTO_NEAR   0x0405
#define NTF_START_PRESS 0x0406
#define NTF_JS1BT_PRESS 0x0407
#define NTF_JS1D0_PRESS 0x0408
#define NTF_JS1D1_PRESS 0x0409
#define NTF_JS1D2_PRESS 0x040a
#define NTF_JS1D3_PRESS 0x040b
#define NTF_JS2BT_PRESS 0x040c
#define NTF_JS2D0_PRESS 0x040d
#define NTF_JS2D1_PRESS 0x040e
#define NTF_JS2D2_PRESS 0x040f
#define NTF_JS2D3_PRESS 0x0410
#define NTF_RXCOL_TRIG  0x0411
#define NTF_MOTO_EMER   0x0411
void report_notify(unsigned short notify);
#endif