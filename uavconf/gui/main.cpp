#include <QtWidgets>
#include "mainwidget.h"

#if defined(ANDROID)
#include <QtCore/private/qandroidextras_p.h>
#endif

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);

#if defined(ANDROID)
	main_widget w;
	
	QtAndroidPrivate::requestPermission(QString("android.permission.WRITE_EXTERNAL_STORAGE"));
#else
	char *uartdev;

	uartdev = (argc > 1) ? argv[1] : NULL;
	
	main_widget w(uartdev);

	w.resize(1200, 800);
#endif

	w.show();

	return a.exec();
}
