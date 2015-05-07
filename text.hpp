#pragma once
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

inline void DrawText(char* text, float x, float y, bool blue = false) {
	float current_color[4];
	glGetFloatv(GL_CURRENT_COLOR, current_color);
	if (blue) {
		glColor3f(0.34, 0.667, 0.864);
	} else {
		glColor3f(0., 0.433, 0.);
		glRasterPos2f(x+1.5, y-1.5);
		for (int i = 0; text[i]; i++)
			glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
		glColor3f(0., 0.867, 0.);
	}
	glRasterPos2f(x, y);
	for (int i = 0; text[i]; i++)
		glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, text[i]);
	glColor3fv(current_color);
}