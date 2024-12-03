#ChargingInfo.msg

float32 voltage # [V]
float32 current # [A]
uint16 status
uint16 avg_time_to_full # [min]
uint16 avg_time_to_empty # [min]
float32 state_of_charge # charged state [%]
float32 state_of_health # battery health [%]
float32 remain_current # [A]
float32 remain_energy # [Wh]
float32 temperature # ['C]