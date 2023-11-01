void setup() {
  // using timer 1
  DDRB = 1<<DDB1; // PB1 as output
  TCCR1 = 1<<CTC1 | 1<<COM1A0 | 1<<CS11; // CTC mode, toggle OC1A (PB1), divide by 2 prescaler
  OCR1C = 226; // 38 kHz   32,000,000 / (2*2*(1+210))


  // using timer 0
  // DDRB = 1<<DDB0; // PB0 as output
  // TCCR0A = 1<<COM0A0 | 1<<WGM01; // toggle OC0A, CTC mode
  // TCCR0B = 1<<CS00; // no prescaling
  // OCR0A = 52; // 37.7 kHz   4,000,000 / (2*1*(1+52))
}

void loop() {

}