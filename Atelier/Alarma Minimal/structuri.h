typedef struct
{
  int pin; // pinul de legatura
  int stare; // in functiune sau nu
  int tip_senzor; // nealocat, reed, pir, fum, gaze
  PGM_P name; // numele in clar
  bool activ; // activat, dezactivat ()
  bool alarmat;
} senzor;

