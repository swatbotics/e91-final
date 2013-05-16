#include "biped.h"

using namespace std;

int main(int argc, char** argv){
	biped a = biped(10,10,0,LEFT);
	biped b = biped(13,13,0,RIGHT);
	biped c = biped(16,10,0,LEFT);
	biped d = biped(19,13.5,15,RIGHT);
	biped e = biped(21.5,11,15,LEFT);
	float width = 4;
	float height = 2;
	float scale = 5;
	
	FILE* svg = fopen("test.svg", "w");
	
	fprintf(svg, "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
    fprintf(svg, "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n");
    fprintf(svg, "  \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n");
    fprintf(svg, "<svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\"\n");
    fprintf(svg, "     xmlns:xlink=\"http://www.w3.org/1999/xlink\"\n");
    fprintf(svg, "     width=\"%upx\" height=\"%upx\" viewBox=\"0 0 %u %u\" >\n", 
          200, 100, 200, 100);
		  
	a.draw(svg, scale, width, height);
	b.draw(svg, scale, width, height);
	c.draw(svg, scale, width, height);
	d.draw(svg, scale, width, height);
	e.draw(svg, scale, width, height);
	fprintf(svg, "</svg>\n");
    fclose(svg);

	return 0;
}
	