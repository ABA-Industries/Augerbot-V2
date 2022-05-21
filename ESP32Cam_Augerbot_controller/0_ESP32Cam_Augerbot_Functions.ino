void send_serial_command()
{
    //append D to the front so the Arduino Due knows its the beginning of the command string
  serial_command_buffer.concat("D");
  for (int i = 0; i < (sizeof(motor_direction_array) / sizeof(int)); i++)
  {
    int temp = motor_direction_array[i] * dc_motor_speed_rpm;
    serial_command_buffer.concat(temp);
  }

  //send the final serial command string appended with servo speed
  if (serial_command_buffer.length() > 0 && serial_command_buffer != serial_command_buffer_old )
  {
    Serial.println(serial_command_buffer);
  }

  //then reset the string for new command
  serial_command_buffer_old = serial_command_buffer;
  serial_command_buffer = "";

  old_Packet[0] = incomingPacket[0];
}

void update_motor_speed ()
{
  /////////////////////////////////////////////
  //update motor speed
  if (incomingPacket[0] == '1')
  {
    dc_motor_speed_rpm = 1;
    dc_motor_speed_rpm_old = dc_motor_speed_rpm;
    incomingPacket[0] = old_Packet[0];
  }

  if (incomingPacket[0] == '2')
  {
    dc_motor_speed_rpm = 2;
    dc_motor_speed_rpm_old = dc_motor_speed_rpm;
    incomingPacket[0] = old_Packet[0];
  }

  if (incomingPacket[0] == '3')
  {
    dc_motor_speed_rpm = 3;
    dc_motor_speed_rpm_old = dc_motor_speed_rpm;
    incomingPacket[0] = old_Packet[0];
  }

  if (incomingPacket[0] == '4')
  {
    dc_motor_speed_rpm = 4;
    dc_motor_speed_rpm_old = dc_motor_speed_rpm;
    incomingPacket[0] = old_Packet[0];
  }

  if (incomingPacket[0] == '5')
  {
    dc_motor_speed_rpm = 5;
    dc_motor_speed_rpm_old = dc_motor_speed_rpm;
    incomingPacket[0] = old_Packet[0];
  }

  if (incomingPacket[0] == '6')
  {
    dc_motor_speed_rpm = 6;
    dc_motor_speed_rpm_old = dc_motor_speed_rpm;
    incomingPacket[0] = old_Packet[0];
  }

  if (incomingPacket[0] == '7')
  {
    dc_motor_speed_rpm = 7;
    dc_motor_speed_rpm_old = dc_motor_speed_rpm;
    incomingPacket[0] = old_Packet[0];
  }

  if (incomingPacket[0] == '8')
  {
    dc_motor_speed_rpm = 8;
    dc_motor_speed_rpm_old = dc_motor_speed_rpm;
    incomingPacket[0] = old_Packet[0];
  }

  if (incomingPacket[0] == '9')
  {
    dc_motor_speed_rpm = 9;
    dc_motor_speed_rpm_old = dc_motor_speed_rpm;
    incomingPacket[0] = old_Packet[0];
  }
}

void update_motor_direction()
{
  // relative to front
  // 1 = right
  // -1 = left

  //{M1,  M2,
  // M3 , M4}

  //remember the two back motors are inverted

  if (incomingPacket[0] == 'S')
  {
    //Stop
    dc_motor_speed_rpm = 0;
  }



  if (incomingPacket[0] == 'W')
  {
    //forward
    motor_direction_array[0] = 1;
    motor_direction_array[1] = -1;
    motor_direction_array[2] = 1;
    motor_direction_array[3] = -1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  if (incomingPacket[0] == 'X')
  {
    //reverse
    motor_direction_array[0] = -1;
    motor_direction_array[1] = 1;
    motor_direction_array[2] = -1;
    motor_direction_array[3] = 1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  if (incomingPacket[0] == 'A')
  {
    //strafe left
    motor_direction_array[0] = -1;
    motor_direction_array[1] = -1;
    motor_direction_array[2] = 1;
    motor_direction_array[3] = 1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  if (incomingPacket[0] == 'D')
  {
    //strafe right
    motor_direction_array[0] = 1;
    motor_direction_array[1] = 1;
    motor_direction_array[2] = -1;
    motor_direction_array[3] = -1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  if (incomingPacket[0] == 'Q')
  {
    //rotate left
    motor_direction_array[0] = -1;
    motor_direction_array[1] = -1;
    motor_direction_array[2] = -1;
    motor_direction_array[3] = -1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }

  if (incomingPacket[0] == 'E')
  {
    //rotate right
    motor_direction_array[0] = 1;
    motor_direction_array[1] = 1;
    motor_direction_array[2] = 1;
    motor_direction_array[3] = 1;

    dc_motor_speed_rpm = dc_motor_speed_rpm_old;
  }
}
