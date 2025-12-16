#include <QtWidgets>
#include "mainwidget.h"

#if defined(ANDROID)
#include <QtCore/private/qandroidextras_p.h>
#endif

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	main_widget w;

#if defined(ANDROID)
    QtAndroidPrivate::requestPermission(QString("android.permission.WRITE_EXTERNAL_STORAGE"));
#else
	w.resize(1200, 800);
#endif

	w.show();

	return a.exec();
}
