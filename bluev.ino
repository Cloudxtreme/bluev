extern "C" {
  extern void hwsetup(void);
  extern void hwloop(void);
}
/*========================================================================*/
void loop(void)
{
  hwloop();
}
/*-------------------------------------------------------------------------*/
void setup(void)
{
  hwsetup();
}

