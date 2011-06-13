/**
 * Author: W. van Teijlingen <wouter.vanteijlingen@student.hu.nl>
 *
 * This program takes an AutoCAD generated X,Y,Z file and creates a
 * XML configuration file with proper parameters.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

int main(int argc, char* argv[])
{
	FILE* in  = fopen(argv[1], "r");
	FILE* out = fopen(argv[2], "w");
	
	fprintf(out, "<?xml version=\"1.0\" ?>\n");
	fprintf(out, "<config>\n");
	fprintf(out, "<option number=\"4\">\n");

        double x, y, z;
	int i = 1;
	while( fscanf(in, "%lg,%lg,%lg", &x, &y, &z) != EOF)
	{
		printf("%g,%g,%g\n", x, y, z);
		fprintf(out, "\t<row number=\"%d\">\n", i);
		
		fprintf(out, "\t\t<X>%lg</X>\n", x);
		fprintf(out, "\t\t<Y>%lg</Y>\n", y);
		fprintf(out, "\t\t<Z>%lg</Z>\n", z);
		fprintf(out, "\t\t<mode>single</mode>\n");

		fprintf(out, "\t</row>\n");
		i++;
	}

	fprintf(out, "</option>\n");
	fprintf(out, "</config>\n");

	close(in);
	close(out);

	return 0;
}
