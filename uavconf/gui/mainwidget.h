#ifndef MAINWIDGET_H
#define MAINWIDGET_H

#include <sys/socket.h>
#include <arpa/inet.h>

#include <string>
#include <vector>
#include <QWidget>
#include <QObject>
#include <QtWidgets>


using namespace std;

enum SETTING_TYPE {
	SETTING_TYPE_FLOAT,
	SETTING_TYPE_MODE,
	SETTING_TYPE_BUTTON
};

class commands_tree;
class main_widget;

class setting : public QWidget
{
	Q_OBJECT

public:
	setting(QWidget *parent = 0,
		string n = string(""),
		string c = string(""),
		commands_tree *cmdstree = nullptr);
	~setting();

	string &get_name() { return name; }
	string &get_command() { return command; }
	QWidget *get_label() { return label; }

	virtual QWidget *get_field() = 0;
	virtual string get_value() const { return string(""); }
	virtual void set_value(const string &s) { (void) s; }

private:
	QLabel *label;
	string name;
	string command;
};

class float_setting : public setting
{
public:
	float_setting(QWidget *parent = 0,
		string n = string(""),
		string c = string(""),
		commands_tree *cmdstree = nullptr);
	~float_setting();

	QWidget *get_field() { return edit; }
	string get_value() const;
	void set_value(const string &s);

private:
	QLineEdit *edit;
	QRegExpValidator *validator;
};

class uint_setting : public setting
{
public:
	uint_setting(QWidget *parent = 0,
		string n = string(""),
		string c = string(""),
		commands_tree *cmdstree = nullptr);
	~uint_setting();

	QWidget *get_field() { return edit; }
	string get_value() const;
	void set_value(const string &s);

private:
	QLineEdit *edit;
	QRegExpValidator *validator;
};

class button_setting : public setting
{
public:
	button_setting(QWidget *parent = 0,
		string n = string(""),
		void (*eh)(void *) = nullptr,
		void *ua = nullptr);
	~button_setting();

	QWidget *get_field() { return button; }

private:
	void click_handler();

	QPushButton *button;

	void (*external_handler)(void *);
	void *userdata;
};

class mode_setting : public setting
{
public:
	mode_setting(QWidget *parent = 0,
		string n = string(""),
		string c = string(""),
		commands_tree *cmdstree = nullptr,
		vector<string> modes = vector<string>(),
		string init = string());
	~mode_setting();

	QWidget *get_field() { return box; }
	string get_value() const;
	void set_value(const string &s);

private:
	int def_idx;
	QComboBox *box;	
};

class settings_group : public QWidget
{
	Q_OBJECT

public:
	settings_group(QWidget *parent = 0, string n = string(""),
		bool nsb = false,
		main_widget *mw = nullptr);
	~settings_group();

	main_widget *get_main_widget() { return _main_widget; }

	void add_setting(setting *s, bool addlabel = true);
	void add_setting_pair(setting *s1, setting *s2);

	string get_name() { return name; }
	
	map<string, setting *> &get_settings() { return settings; }
	
	setting *get_setting(const string &n) { return settings[n]; }
	
private:
	string name;
	QGroupBox *group;
	QGridLayout *layout;
	main_widget *_main_widget;
	bool has_send_button;
	
	int layout_last;
	
	map<string, setting *> settings;
};

class float_settings_group : public settings_group
{
public:
	float_settings_group(QWidget *parent = 0,
		string name = string(""),
		vector<string> s = vector<string>(),
		vector<string> c = vector<string>(),
		commands_tree *cmdstree = nullptr,
		bool nsb = false,
		main_widget *mw = nullptr);
};

class uint_settings_group : public settings_group
{
public:
	uint_settings_group(QWidget *parent = 0,
		string name = string(""),
		vector<string> s = vector<string>(),
		vector<string> c = vector<string>(),
		commands_tree *cmdstree = nullptr,
		bool nsb = false,
		main_widget *mw = nullptr);
};

class mode_settings_group : public settings_group
{
public:
	mode_settings_group(QWidget *parent = 0,
		string name = string(""),
		vector<string> s = vector<string>(),
		vector<string> c = vector<string>(),
		commands_tree *cmdstree = nullptr,
		vector<string> modes = vector<string>(),
		string init = string(""), 
		bool nsb = false,
		main_widget *mw = nullptr);
};

class settings_tab : public QWidget
{
	Q_OBJECT

public:
	settings_tab(QWidget *parent = 0);
	~settings_tab();

	void add_group(settings_group *s, int r, int c,
		int rs = 1, int cs = 1);
	void add_setting(setting *s, int r, int c,
		int rs = 1, int cs = 1);
	void add_widget(QWidget *w, int r, int c,
		int rs = 1, int cs = 1);

	settings_group *get_group(string name) { return groups[name]; }
	const setting *get_setting(string name) { return settings[name]; }

	map<string, settings_group *>  &get_groups() { return groups; }

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

	QLineEdit *get_line() { return line; }
private:
	QGridLayout *grid;
	QTextEdit *edit;
	QLineEdit *line;
};

class commands_tree {
public:
	commands_tree(const string &s = "");
	~commands_tree();

	commands_tree *get_child(const string &s);

	setting *get_setting() { return cmdsetting; }

	void set_setting(setting *s);

	map<string, commands_tree *> &get_children() { return children; }

private:
	string name;
	map<string, commands_tree *> children;
	setting *cmdsetting;
};

class main_widget : public QWidget
{
	Q_OBJECT
public:
	main_widget(QWidget *parent = 0);
	~main_widget();

	void write_to_terminal(string s);

	void string_to_conf(const string &);
	string conf_to_string();

	void start_catch_uav_out() { catchuavout = true; }
	void stop_catch_uav_out() { catchuavout = false; }

	void send_uav_conf_cmd(string cmd);
	void send_uav_info_cmd(string cmd);
	void send_uav_get_cmd(string cmd, string *out);
	void get_uav_log(string &s);

	map<string, settings_tab *> get_tabs() { return tabs; }
	
public slots:
	void record_size_item_changed(int idx);
	void lt_item_changed(int idx);
	void lb_item_changed(int idx);
	void rt_item_changed(int idx);
	void rb_item_changed(int idx);

protected:
	void keyPressEvent(QKeyEvent *event) override;

private:	
	void return_pressed();

	void timer_handler();

	void update_motors_mapping(int motoridx);

	bool catchuavout;
	struct sockaddr_in rsi;
	int lsfd;

	commands_tree *cmdstree;

	map<string, settings_tab *> tabs;
	map<string, setting *> settings;

	terminal *term;
	QTabWidget *tab;
	QGridLayout *grid;
	QTimer *timer;
};

#endif
