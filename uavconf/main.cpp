#include <QtWidgets>
#include "mainwidget.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

	main_widget w;

	w.resize(1200, 700);
	w.show();

	return a.exec();
}
