#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <sys/socket.h>
#include <arpa/inet.h>

#include <string>
#include <vector>
#include <QWidget>

using namespace std;

enum SETTING_TYPE {
	SETTING_TYPE_FLOAT,
	SETTING_TYPE_MODE,
	SETTING_TYPE_BUTTON
};

class setting : public QWidget
{
public:
	setting(QWidget *parent = 0,
		enum SETTING_TYPE t = SETTING_TYPE_FLOAT,
		string n = string(""),
		string c = string(""),
		vector<string> modes = vector<string>());
	~setting();

	string &get_name();
	string &get_command();
	QWidget *get_label();
	QWidget *get_field();
	string get_value() const;
	void set_value(const string &s);

private:
	QLabel *label;
	string name;
	string command;
	
	union {
		struct {
			QLineEdit *edit;
			QRegExpValidator *validator;
		};

		QComboBox *box;
		QPushButton *button;
	};

	enum SETTING_TYPE type;
};

class settings_group : public QWidget
{
public:
	settings_group(QWidget *parent = 0, string n = string(""));
	~settings_group();

	void add_setting(setting *s);
	string get_setting_value(string name) const;

	string get_name();
	
	void set_setting_value(string name, const string &v);

	map<string, setting *> &get_settings();
	
private:
	string name;
	QGroupBox *group;
	QGridLayout *layout;
	
	int layout_last;
	
	map<string, setting *> settings;
};

class float_settings_group : public settings_group
{
public:
	float_settings_group(QWidget *parent = 0,
		string name = string(""),
		vector<string> s = vector<string>(),
		vector<string> c = vector<string>());
};

class settings_tab : public QWidget
{
public:
	settings_tab(QWidget *parent = 0);
	~settings_tab();

	void add_group(settings_group *s, int r, int c,
		int rs = 1, int cs = 1);
	void add_setting(setting *s, int r, int c, int
		rs = 1, int cs = 1);

	settings_group *get_group(string name);
	const setting *get_setting(string name);

	map<string, settings_group *>  &get_groups();

private:
	QGridLayout *grid;

	map<string, settings_group *> groups;
	map<string, setting *> settings;
};

class terminal : public QWidget
{
public:
	terminal(QWidget *parent = 0);
	~terminal();

	void add_output(string s);

private:
	QGroupBox *group;
	QGridLayout *grid;
	QTextEdit *edit;
	QLineEdit *line;
};

class main_widget : public QWidget
{
public:
	main_widget(QWidget *parent = 0);
	~main_widget();

private:
	void conf_to_string();
	void string_to_conf(const string &);

	void open_click_handler();
	void save_click_handler();
	void connect_click_handler();
	void flash_click_handler();

	struct sockaddr_in rsi;
	int lsfd;

	string conf;

	map<string, settings_tab *> tabs;
	map<string, setting *> settings;

	terminal *term;
	QTabWidget *tab;
	QGridLayout *grid;
};

#endif
