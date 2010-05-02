#include <stdio.h>
#include "gpc.h"

int main(void)
{
  gpc_polygon subject, clip, result;
  FILE *sfp, *cfp, *ofp;
    
  sfp= fopen("poly1", "r");
  cfp= fopen("poly2", "r");
  gpc_read_polygon(sfp, 0, &subject);
  gpc_read_polygon(cfp, 0, &clip);
    
  //gpc_polygon_clip(GPC_INT, &subject, &clip, &result);
  //gpc_polygon_clip(GPC_DIFF, &subject, &clip, &result);
  //gpc_polygon_clip(GPC_XOR, &subject, &clip, &result);
  gpc_polygon_clip(GPC_UNION, &subject, &clip, &result);

  ofp= fopen("result", "w");
  gpc_write_polygon(ofp, 0, &result);
  printf("result written to \"result\" file\n");
  gpc_free_polygon(&subject);
  gpc_free_polygon(&clip);
  gpc_free_polygon(&result);
          
  fclose(sfp);
  fclose(cfp);
  fclose(ofp);
  return 0;
}
