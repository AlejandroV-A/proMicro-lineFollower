# 1 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
// Machine God [G4]
# 3 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 2

// TODO tunning the gains, and setting translational speed
// with speed ups and downs. Do not start any motor until a line is detected.

void setup(void) {
  setupADC(2, A6, A8, A9, A10, A0, A1, A2, A3);
  ddwmrInit(0, 15, 14, 7, 16, 5, 6);
  // stop, leftFront, leftBack, rightFront, rightBack, leftPWM,  rightPWM
  startADC();
  pinMode(1, 0x1);
  pinMode(3, 0x1);
}

void loop(void) {
  pidPool();
  if (error == 0) {
    ((1 == 0) ? ((*( (volatile uint8_t *)( 
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (1)
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) &= ~( 
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (1)
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))) : ((*( (volatile uint8_t *)( 
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (1)
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) |= ( 
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (1)
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 19 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))));
    ((1 == 0) ? ((*( (volatile uint8_t *)( 
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (3)
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) &= ~( 
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (3)
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))) : ((*( (volatile uint8_t *)( 
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (3)
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) |= ( 
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (3)
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 20 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))));
  } else if (error < 0) {
    ((1 == 0) ? ((*( (volatile uint8_t *)( 
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (1)
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) &= ~( 
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (1)
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))) : ((*( (volatile uint8_t *)( 
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (1)
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) |= ( 
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (1)
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 22 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))));
    ((0 == 0) ? ((*( (volatile uint8_t *)( 
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (3)
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) &= ~( 
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (3)
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))) : ((*( (volatile uint8_t *)( 
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (3)
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) |= ( 
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (3)
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 23 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))));
  } else {
    ((0 == 0) ? ((*( (volatile uint8_t *)( 
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (1)
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) &= ~( 
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (1)
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))) : ((*( (volatile uint8_t *)( 
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (1)
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) |= ( 
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (1)
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 25 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))));
    ((1 == 0) ? ((*( (volatile uint8_t *)( 
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (3)
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) &= ~( 
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (3)
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))) : ((*( (volatile uint8_t *)( 
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   port_to_output_PGM + (( 
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_port_PGM + (3)
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint16_t __result; __asm__ __volatile__ ( "lpm %A0, Z+" "\n\t" "lpm %B0, Z" "\n\t" : "=r" (__result), "=z" (__addr16) : "1" (__addr16) ); __result; }))
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ) ) |= ( 
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   (__extension__({ uint16_t __addr16 = (uint16_t)((uint16_t)(
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   digital_pin_to_bit_mask_PGM + (3)
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino" 3
   )); uint8_t __result; __asm__ __volatile__ ( "lpm %0, Z" "\n\t" : "=r" (__result) : "z" (__addr16) ); __result; })) 
# 26 "/home/alejandrov/Dropbox/Arduino/Seguidor_micro/Seguidor_micro.ino"
   ))));
  }
}

/* Pro micro pinout
 AVR // Analog // Com // PWM // Arduino // Use
 d3       tx          1  [LED_R]
 d2       rx         *0  [STOP]
 d1       SDA        *2  [ENABLE]
 d0       SCL P0b976  3  [LED_L]
 d4  *A6              4  [Sens]
 c6          *P3a488  5  [PWM]
 d7   A7     *P4a488  6  [PWM]
 e6                  *7  [Car]
 b4  *A8              8  [Sens]
 b5  *A9      P1a488  9  [Sens]
 b6  *A10     P1b488 10  [Sens]
 b2       MOSI      *16  [Car]
 b3       MISO      *14  [Car]
 b1       SCLK      *15  [Car]
 f7  *A0             18  [Sens]
 f6  *A1             19  [Sens]
 f5  *A2             20  [Sens]
 f4  *A3             21  [Sens]
           RXLED      17  [RXLED]
//*/
/*
I'm a Ph.D. candidate that has been interested in embedded systems for more than
10 years.

My first approach to embedded system development was as a high school student,
where I learned to program Microchip PIC MCUs in assembler language, at this
time starting to participate in robotics contests. In my engineering studies, I
also learned to program AVR MCUs in assembler language, to then advance to
program both Microchip's platforms in C language, continuing to participate and
win in international robotics contest.

As a graduate student, I've been working with embedded systems to tests the
control laws that I propose. The embedded systems that I use are unmanned
vehicles, mainly quadcopters. In this works, I also started using NXP LPC MCUs
in C language.

Nowadays, I'm also lecturing the "embedded systems" subject at UNITEC. In this
university, as the students already own some hardware, particularly Arduino, I
had to fit the program to use that hardware. Arduino is a platform that I had
never used before, but transporting the knowledge from other platforms I
understood its nuances swiftly.

This background shows that the embedded systems my main technological interest
and where I would like to perform for my career, using my high level of
technical understanding as well as the PhD-level abilities in propose, adapt,
and use state-of-the-art technology.

This background makes the embedded systems my main technological interest and
where I would like to perform for my career, using my high level of technical
understanding as well as the many abilities in proposing, adapting, and using
state-of-the-art technology.
*/