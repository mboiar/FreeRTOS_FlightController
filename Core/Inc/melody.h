#pragma once

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_DB3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_EB3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST     0
#define NOTE_REST 0
#define REST 0

typedef struct {
    uint16_t freq;   // 0 = rest
    uint16_t dur_ms;
} Tone;

Tone ode_to_joy[] = {
    {NOTE_E4, 4}, {NOTE_E4, 4}, {NOTE_F4, 4}, {NOTE_G4, 4},
    {NOTE_G4, 4}, {NOTE_F4, 4}, {NOTE_E4, 4}, {NOTE_D4, 4},
    {NOTE_C4, 4}, {NOTE_C4, 4}, {NOTE_D4, 4}, {NOTE_E4, 4},
    {NOTE_E4, 4}, {NOTE_D4, 4}, {NOTE_D4, 2}, {0, 8},  // rest

    {NOTE_E4, 4}, {NOTE_E4, 4}, {NOTE_F4, 4}, {NOTE_G4, 4},
    {NOTE_G4, 4}, {NOTE_F4, 4}, {NOTE_E4, 4}, {NOTE_D4, 4},
    {NOTE_C4, 4}, {NOTE_C4, 4}, {NOTE_D4, 4}, {NOTE_E4, 4},
    {NOTE_D4, 4}, {NOTE_C4, 4}, {NOTE_C4, 2}, {0, 1000}   // rest
};

Tone imperial[] = {
     {440, 500}, {440, 500}, {440, 500}, // A A A
    {349, 350}, {523, 150},             // F C5
    {440, 500}, {349, 350}, {523, 150}, // A F C5
    {440, 1000},                         // A
    {659, 500}, {659, 500}, {659, 500}, // E E E
    {698, 350}, {523, 150},             // F C5
    {415, 500}, {349, 350}, {523, 150}, // G# F C5
    {440, 1000},                         // A

    // second part
    {880, 500}, {440, 350}, {440, 150}, // A5 A A
    {880, 500}, {830, 250}, {784, 250}, // A5 G#5 G5
    {740, 125}, {698, 125}, {740, 250}, // F# G F#
    {0, 250},                           // Rest
    {415, 250}, {554, 500}, {523, 500}, // G# C#5 C5
    {440, 500},                         // A
    {0, 400}
};

Tone uiaa[] = {
     {NOTE_C4, 4},  // U
    {0,       8},  // short rest
    {NOTE_E4, 4},  // I
    {0,       8},  // short rest
    {NOTE_A4, 4},  // A
    {NOTE_A4, 4},  // A again
    {0,       4}   // rest to finish
};

int nokia[] = {
  NOTE_E5, NOTE_D5, NOTE_FS4, NOTE_GS4, 
  NOTE_CS5, NOTE_B4, NOTE_D4, NOTE_E4, 
  NOTE_B4, NOTE_A4, NOTE_CS4, NOTE_E4,
  NOTE_A4
};

int nokia_durations[] = {
  8, 8, 4, 4,
  8, 8, 4, 4,
  8, 8, 4, 4,
  2
};

int starwars[] = {
  NOTE_AS4, NOTE_AS4, NOTE_AS4,
  NOTE_F5, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_AS5, NOTE_G5, NOTE_C5, NOTE_C5, NOTE_C5,
  NOTE_F5, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,

  NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F6, NOTE_C6,
  NOTE_AS5, NOTE_A5, NOTE_AS5, NOTE_G5, NOTE_C5, NOTE_C5,
  NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5,
  NOTE_F5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_D5, NOTE_E5, NOTE_C5, NOTE_C5,
  NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5,

  NOTE_C6, NOTE_G5, NOTE_G5, REST, NOTE_C5,
  NOTE_D5, NOTE_D5, NOTE_AS5, NOTE_A5, NOTE_G5, NOTE_F5,
  NOTE_F5, NOTE_G5, NOTE_A5, NOTE_G5, NOTE_D5, NOTE_E5, NOTE_C6, NOTE_C6,
  NOTE_F6, NOTE_DS6, NOTE_CS6, NOTE_C6, NOTE_AS5, NOTE_GS5, NOTE_G5, NOTE_F5,
  NOTE_C6
};

int starwars_durations[] = {
  8, 8, 8,
  2, 2,
  8, 8, 8, 2, 4,
  8, 8, 8, 2, 4,
  8, 8, 8, 2, 8, 8, 8,
  2, 2,
  8, 8, 8, 2, 4,

  8, 8, 8, 2, 4,
  8, 8, 8, 2, 8, 16,
  4, 8, 8, 8, 8, 8,
  8, 8, 8, 4, 8, 4, 8, 16,
  4, 8, 8, 8, 8, 8,

  8, 16, 2, 8, 8,
  4, 8, 8, 8, 8, 8,
  8, 8, 8, 4, 8, 4, 8, 16,
  4, 8, 4, 8, 4, 8, 4, 8,
  1
};

int tokio_melody[] = {
  NOTE_AS4, REST, NOTE_AS4, REST, NOTE_AS4, REST, NOTE_AS4, REST,
  NOTE_AS4, NOTE_B4, NOTE_DS5,
  NOTE_AS4, REST, NOTE_AS4, REST,
  NOTE_AS4, NOTE_B4, NOTE_DS5,
  NOTE_AS4, REST, NOTE_AS4, REST,
  NOTE_AS4, NOTE_B4, NOTE_DS5,
  NOTE_AS4, REST, NOTE_AS4, REST,
  NOTE_AS4, NOTE_B4, NOTE_DS5,
  NOTE_F5, REST, NOTE_F5, REST,
  NOTE_GS5, NOTE_FS5, NOTE_F5,
  NOTE_AS4, REST, NOTE_AS4, REST,
  NOTE_GS5, NOTE_FS5, NOTE_F5,
  NOTE_AS4, REST, NOTE_AS4, REST,
  NOTE_AS4, NOTE_B4, NOTE_DS5,
  NOTE_AS4, REST, NOTE_AS4, REST,
  NOTE_AS4, NOTE_B4, NOTE_DS5,
  NOTE_AS4, REST, NOTE_AS4, REST,
  REST
};

int tokio_durations[] = {
  4, 4, 4, 4, 4, 4, 4, 4,
  3, 3, 4,
  4, 4, 4, 4,
  3, 3, 4,
  4, 4, 4, 4,
  3, 3, 4,
  4, 4, 4, 4,
  3, 3, 4,
  4, 4, 4, 4,
  3, 3, 4,
  4, 4, 4, 4,
  3, 3, 4,
  4, 4, 4, 4,
  3, 3, 4,
  4, 4, 4, 4,
  3, 3, 4,
  4, 4, 4, 4,
  1
};

Tone canon_melody[] = {

  // Cannon in D - Pachelbel
  // Score available at https://musescore.com/user/4710311/scores/1975521
  // C F
  {NOTE_FS4,2}, {NOTE_E4,2},
  {NOTE_D4,2}, {NOTE_CS4,2},
  {NOTE_B3,2}, {NOTE_A3,2},
  {NOTE_B3,2}, {NOTE_CS4,2},
  {NOTE_FS4,2}, {NOTE_E4,2},
  {NOTE_D4,2}, {NOTE_CS4,2},
  {NOTE_B3,2}, {NOTE_A3,2},
  {NOTE_B3,2}, {NOTE_CS4,2},
  {NOTE_D4,2}, {NOTE_CS4,2},
  {NOTE_B3,2}, {NOTE_A3,2},
  {NOTE_G3,2}, {NOTE_FS3,2},
  {NOTE_G3,2}, {NOTE_A3,2},

  {NOTE_D4,4}, {NOTE_FS4,8}, {NOTE_G4,8}, {NOTE_A4,4}, {NOTE_FS4,8}, {NOTE_G4,8}, 
  {NOTE_A4,4}, {NOTE_B3,8}, {NOTE_CS4,8}, {NOTE_D4,8}, {NOTE_E4,8}, {NOTE_FS4,8}, {NOTE_G4,8}, 
  {NOTE_FS4,4}, {NOTE_D4,8}, {NOTE_E4,8}, {NOTE_FS4,4}, {NOTE_FS3,8}, {NOTE_G3,8},
  {NOTE_A3,8}, {NOTE_G3,8}, {NOTE_FS3,8}, {NOTE_G3,8}, {NOTE_A3,2},
  {NOTE_G3,4}, {NOTE_B3,8}, {NOTE_A3,8}, {NOTE_G3,4}, {NOTE_FS3,8}, {NOTE_E3,8}, 
  {NOTE_FS3,4}, {NOTE_D3,8}, {NOTE_E3,8}, {NOTE_FS3,8}, {NOTE_G3,8}, {NOTE_A3,8}, {NOTE_B3,8},

  {NOTE_G3,4}, {NOTE_B3,8}, {NOTE_A3,8}, {NOTE_B3,4}, {NOTE_CS4,8}, {NOTE_D4,8},
  {NOTE_A3,8}, {NOTE_B3,8}, {NOTE_CS4,8}, {NOTE_D4,8}, {NOTE_E4,8}, {NOTE_FS4,8}, {NOTE_G4,8}, {NOTE_A4,2},
  {NOTE_A4,4}, {NOTE_FS4,8}, {NOTE_G4,8}, {NOTE_A4,4},
  {NOTE_FS4,8}, {NOTE_G4,8}, {NOTE_A4,8}, {NOTE_A3,8}, {NOTE_B3,8}, {NOTE_CS4,8},
  {NOTE_D4,8}, {NOTE_E4,8}, {NOTE_FS4,8}, {NOTE_G4,8}, {NOTE_FS4,4}, {NOTE_D4,8}, {NOTE_E4,8},
  {NOTE_FS4,8}, {NOTE_CS4,8}, {NOTE_A3,8}, {NOTE_A3,8},

  {NOTE_CS4,4}, {NOTE_B3,4}, {NOTE_D4,8}, {NOTE_CS4,8}, {NOTE_B3,4},
  {NOTE_A3,8}, {NOTE_G3,8}, {NOTE_A3,4}, {NOTE_D3,8}, {NOTE_E3,8}, {NOTE_FS3,8}, {NOTE_G3,8},
  {NOTE_A3,8}, {NOTE_B3,4}, {NOTE_G3,4}, {NOTE_B3,8}, {NOTE_A3,8}, {NOTE_B3,4},
  {NOTE_CS4,8}, {NOTE_D4,8}, {NOTE_A3,8}, {NOTE_B3,8}, {NOTE_CS4,8}, {NOTE_D4,8}, {NOTE_E4,8},
  {NOTE_FS4,8}, {NOTE_G4,8}, {NOTE_A4,2}
};

int hb_melody[] = {
  NOTE_C4, NOTE_C4, 
  NOTE_D4, NOTE_C4, NOTE_F4,
  NOTE_E4, NOTE_C4, NOTE_C4, 
  NOTE_D4, NOTE_C4, NOTE_G4,
  NOTE_F4, NOTE_C4, NOTE_C4,
  
  NOTE_C5, NOTE_A4, NOTE_F4, 
  NOTE_E4, NOTE_D4, NOTE_AS4, NOTE_AS4,
  NOTE_A4, NOTE_F4, NOTE_G4,
  NOTE_F4
};

int hb_durations[] = {
  4, 8, 
  4, 4, 4,
  2, 4, 8, 
  4, 4, 4,
  2, 4, 8,
  
  4, 4, 4, 
  4, 4, 4, 8,
  4, 4, 4,
  2
};
