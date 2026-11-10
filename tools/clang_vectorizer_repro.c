#include <stdio.h>
#include <stdint.h>
typedef struct { int16_t p, q; } cell;
typedef struct { cell cells[64]; int count; } shp;
__attribute__((noinline)) void shift(shp *s, int r, int *out) {
  cell root = s->cells[r];
  int p_min = 1 << 20, q_min = 1 << 20, p_max = -(1 << 20), q_max = -(1 << 20);
  for (int i = 0; i < s->count; i++) {
    cell *c = &s->cells[i];
    c->p = (int16_t)(c->p - root.p);
    c->q = (int16_t)(c->q - root.q);
    if (c->p < p_min) p_min = c->p;
    if (c->q < q_min) q_min = c->q;
    if (c->p > p_max) p_max = c->p;
    if (c->q > q_max) q_max = c->q;
  }
  out[0]=p_min; out[1]=q_min; out[2]=p_max; out[3]=q_max;
}
int main(void){ static shp s; s.count=24; for(int i=0;i<24;i++){s.cells[i].p=(int16_t)(i%5); s.cells[i].q=(int16_t)(i/5);} int o[4]; shift(&s,7,o); printf("%d %d %d %d\n",o[0],o[1],o[2],o[3]); }
