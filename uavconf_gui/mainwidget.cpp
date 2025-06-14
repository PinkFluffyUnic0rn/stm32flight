#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <QtWidgets>
#include "mainwidget.h"

extern "C" {
#include "uavconf.h"
}

using namespace std;

void write_term(void *t, const char *s)
{
	terminal *term;

	term = (terminal *) t;

	term->add_output(string(s) + "\n");
	QCoreApplication::processEvents();
}

void write_conf(void *c, const char *s)
{
	string *conf;

	conf = (string *) c;

	*conf += string(s);
}

void open_click_handler(void *arg)
{
	QString name;
	ifstream infile;
	stringstream buf;
	main_widget *mw;
		
	mw = (main_widget *) arg;

	name = QFileDialog::getOpenFileName(mw, "Open file",
		"", "Text files (*.txt)");

	infile.open(name.toStdString());	

	buf << infile.rdbuf();

	mw->string_to_conf(buf.str());
}

void save_click_handler(void *arg)
{
	QString name;
	ofstream outfile;
	string conf;
	main_widget *mw;

	mw = (main_widget *) arg;
	
	name = QFileDialog::getSaveFileName(mw, "Save file", "",
		"Text files (*.txt)");

	conf = mw->conf_to_string();

	outfile.open(name.toStdString());

	outfile << conf;

	outfile.close();
}

void connect_click_handler(void *arg)
{
	string conf;
	map<string, settings_tab *> tabs;
	main_widget *mw;
	
	mw = (main_widget *) arg;
	
	mw->stop_catch_uav_out();
	
	mw->write_to_terminal(string("loading values...\n"));

	tabs = mw->get_tabs();
	for (auto t = begin(tabs); t != end(tabs); ++t) {
		map<string, settings_group *> &groups
			= t->second->get_groups();

		for (auto g = begin(groups); g != end(groups); ++g) {
			map<string, setting *> &sts
				= g->second->get_settings();

			for (auto s = begin(sts); s != end(sts); ++s) {
				if (s->second->get_command().empty())
					continue;

				mw->send_uav_get_cmd("get "
					+ s->second->get_command(),
					&conf);
				mw->write_to_terminal("get "
					+ s->second->get_command()
					+ "\n");
				
				QCoreApplication::processEvents();
			}
		}

	}

	mw->write_to_terminal(string("Connected to UAV"));

	mw->string_to_conf(conf);
	
	mw->start_catch_uav_out();
}

void flash_click_handler(void *arg)
{
	stringstream ss;
	string line;
	string conf;
	main_widget *mw;
	
	mw = (main_widget *) arg;
	
	mw->stop_catch_uav_out();
	
	conf = mw->conf_to_string();

	ss = stringstream(conf);

	while (getline(ss, line, '\n')) {
		mw->send_uav_conf_cmd(line);
		QCoreApplication::processEvents();
	}
	
	mw->start_catch_uav_out();
}

void log_start_click_handler(void *arg)
{
	map<string, settings_tab *> tabs;
	main_widget *mw;
	
	mw = (main_widget *) arg;

	tabs = mw->get_tabs();
	
	mw->send_uav_info_cmd(
		("log set " + tabs["log"]->get_group("Log write")
		 	->get_setting("records count")
			->get_value() + "\n"));
}

void log_stop_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("log set 0\n");
}

void log_read_click_handler(void *arg)
{
	map<string, settings_tab *> tabs;
	main_widget *mw;
	
	mw = (main_widget *) arg;

	tabs = mw->get_tabs();

	((main_widget *) arg)->send_uav_info_cmd(
		("log rget " + tabs["log"]->get_group("Log read")
		 	->get_setting("from")->get_value() + " "
		 + tabs["log"]->get_group("Log read")->get_setting("to")
			->get_value() + "\n"));
}

void log_load_click_handler(void *arg)
{
	QString name;
	ofstream outfile;
	string output;
	main_widget *mw;

	mw = (main_widget *) arg;
	
	mw->stop_catch_uav_out();

	mw->get_uav_log(output);

	name = QFileDialog::getSaveFileName(mw, "Save file", "",
		"Text files (*.txt)");

	outfile.open(name.toStdString());

	outfile << string(output);

	outfile.close();

	mw->start_catch_uav_out();
}

void info_imu_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("info mpu");
}

void info_pid_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("info pid");
}

void info_values_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("info values");
}

void info_mag_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("info qmc");
}

void info_bar_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("info hp");
}

void info_gnss_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("info gnss");
}

void info_devices_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("info dev");
}

void info_control_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("info ctrl");
}

void info_filter_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("info filter");
}

void info_vtx_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("info irc");
}

setting::setting(QWidget *parent, string n, string c,
	commands_tree *cmdstree)
		: QWidget(parent)
{
	commands_tree *tr;
	stringstream ss;
	string tok;

	label = new QLabel();

	name = n;
	command = c;

	if (cmdstree != nullptr) {
		ss = stringstream(command);
		tr = cmdstree;

		while (getline(ss, tok, ' '))
			tr = tr->get_child(tok);
			
		tr->set_setting(this);
	}

	label->setFrameStyle(QFrame::Panel | QFrame::Sunken);
	label->setText(name.c_str());
}

setting::~setting()
{
	delete label;
}

float_setting::float_setting(QWidget *parent, string n, string c,
	commands_tree *cmdstree) : setting(parent, n, c, cmdstree)
{
	edit = new QLineEdit();

	validator = new QRegExpValidator(	
		QRegExp("[-]{0,1}\\d{0,}\\.\\d{0,}"),0);

	edit->setValidator(validator);

	edit->setText(QString::fromStdString("0.0"));
}

float_setting::~float_setting()
{
	delete edit;
	delete validator;
}

string float_setting::get_value() const
{
	if (edit->text().toStdString().empty())
		return string("0.0");
	else
		return edit->text().toStdString();
}

void float_setting::set_value(const string &s)
{
	string t;

	t = s;
	t.erase(t.find_last_not_of("0\r\n") + 1);

	if (t.back() == '.')
		t.pop_back();

	if (t.empty())
		t = "0";
	
	edit->setText(QString::fromStdString(t));
}

uint_setting::uint_setting(QWidget *parent, string n, string c,
	commands_tree *cmdstree) : setting(parent, n, c, cmdstree)
{
	edit = new QLineEdit();

	validator = new QRegExpValidator(QRegExp("\\d{0,}"),0);

	edit->setValidator(validator);

	edit->setText(QString::fromStdString("0"));
}

uint_setting::~uint_setting()
{
	delete edit;
	delete validator;
}

string uint_setting::get_value() const
{
	if (edit->text().toStdString().empty())
		return string("0");
	else
		return edit->text().toStdString();
}

void uint_setting::set_value(const string &s)
{
	string t;

	t = s;
	t.erase(t.find_last_not_of("0\r\n") + 1);

	if (t.back() == '.')
		t.pop_back();

	if (t.empty())
		t = "0";
	
	edit->setText(QString::fromStdString(t));
}

button_setting::button_setting(QWidget *parent, string n,
	void (*eh)(void *), void *ua) : setting(parent, n, "", nullptr)
{

	button = new QPushButton(QString::fromStdString(n));

	if (eh != nullptr) {
		userdata = ua;
		external_handler = eh;

		connect(button, &QPushButton::clicked, this,
			&button_setting::click_handler);
	}
}

button_setting::~button_setting()
{
	delete button;
}

void button_setting::click_handler()
{
	external_handler(userdata);
}

mode_setting::mode_setting(QWidget *parent, string n, string c,
	commands_tree *cmdstree, vector<string> modes)
		: setting(parent, n, c, cmdstree)
{
	box = new QComboBox();

	for (auto it = begin(modes); it != end(modes); ++it)
		box->addItem((*it).c_str());
}

mode_setting::~mode_setting()
{
	delete box;
}

string mode_setting::get_value() const
{
	return box->currentText().toStdString();
}

settings_group::settings_group(QWidget *parent,
	string n) : QWidget(parent), name(n), layout_last(0)
{
	group = new QGroupBox(QString::fromStdString(name),
		dynamic_cast<QWidget *>(this));
	layout = new QGridLayout(group);
}

settings_group::~settings_group()
{
	for (auto it = begin(settings); it != end(settings); ++it)
		delete it->second;

	delete layout;
	delete group;
}

void settings_group::add_setting(setting *s, bool addlabel)
{
	settings[s->get_name()] = s;

	if (addlabel) {
		layout->addWidget(s->get_label(), layout_last, 0);
		layout->addWidget(s->get_field(), layout_last, 1);
	}
	else
		layout->addWidget(s->get_field(), layout_last, 0, 1, 2);

	++layout_last;
}

void settings_group::add_setting_pair(setting *s1, setting *s2)
{
	settings[s1->get_name()] = s1;
	settings[s2->get_name()] = s2;

	layout->addWidget(s1->get_field(), layout_last, 0);
	layout->addWidget(s2->get_field(), layout_last, 1);

	++layout_last;
}

float_settings_group::float_settings_group(QWidget *parent,
	string name, vector<string> s, vector<string> c,
		commands_tree *cmdstree)
		: settings_group(parent, name)
{
	size_t i;

	for (i = 0; i < s.size(); ++i) {
		add_setting(new float_setting(nullptr,
			s[i], c[i], cmdstree));
	}
}

settings_tab::settings_tab(QWidget *parent) : QWidget(parent)
{
	grid = new QGridLayout(this);
}

settings_tab::~settings_tab()
{
	for (auto it = begin(groups); it != end(groups); ++it)
		delete it->second;

	for (auto it = begin(settings); it != end(settings); ++it)
		delete it->second;

	delete grid;
}

void settings_tab::add_group(settings_group *s,
	int r, int c, int rs, int cs)
{
	groups[s->get_name()] = s;
	grid->addWidget(s, r, c, rs, cs);
}

void settings_tab::add_setting(setting *s,
	int r, int c, int rs, int cs)
{
	settings[s->get_name()] = s;
	grid->addWidget(s, r, c, rs, cs);
}

terminal::terminal(QWidget *parent) : QWidget(parent)
{
	grid = new QGridLayout(this);
	edit = new QTextEdit;
	line = new QLineEdit;

	edit->setReadOnly(true);

	grid->addWidget(edit, 0, 0);
	grid->addWidget(line, 1, 0);
}

terminal::~terminal()
{
	delete line;
	delete edit;
	delete grid;
}

void terminal::add_output(string s)
{
	edit->moveCursor(QTextCursor::End);
	edit->insertPlainText(QString::fromStdString(s));
	edit->moveCursor(QTextCursor::End);
}

commands_tree::commands_tree(const string &n) : cmdsetting(nullptr)
{
	name = n;
}

commands_tree::~commands_tree()
{
	for (auto it = begin(children); it != end(children); ++it)
		delete it->second;
}

commands_tree *commands_tree::get_child(const string &s)
{
	if (children.count(s) == 0)
		children[s] = new commands_tree(s);
	
	return children[s];
}

void commands_tree::set_setting(setting *s)
{
	cmdsetting = s;
}

main_widget::main_widget(QWidget *parent)
	: QWidget(parent), catchuavout(true)
{
	grid = new QGridLayout(this);
	tab = new QTabWidget;
	term = new terminal();
	
	cmdstree = new commands_tree;

	initsocks(&lsfd, &rsi);

	tabs["pid"] = new settings_tab;
	tabs["adjustments"] = new settings_tab;
	tabs["filters"] = new settings_tab;
	tabs["control"] = new settings_tab;
	tabs["log"] = new settings_tab;
	tabs["info"] = new settings_tab;
	
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"rate PID", 
		{"P",		"I", 		"D"},
		{"pid stilt p",	"pid stilt i",	"pid stilt d"},
		cmdstree), 0, 0);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"angle PID",
		{"P",		"I",		"D"},
		{"pid tilt p",	"pid tilt i",	"pid tilt d"},
		cmdstree), 0, 1);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"yaw PID",
		{"P",		"I",		"D"},
		{"pid syaw p",	"pid syaw i",	"pid syaw d"},
		cmdstree), 0, 2);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"yaw position PID",
		{"P",		"I",		"D"},
		{"pid yaw p",	"pid yaw i",	"pid yaw d"},
		cmdstree), 0, 3);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"throttle PID",
		{"P",			"I",			"D"},
		{"pid throttle p", 	"pid throttle i",	"pid throttle d"},
		cmdstree), 1, 0);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"climb rate PID",
		{"P",			"I",			"D"},
		{"pid climbrate p",	"pid climbrate i",	"pid climbrate d"},
		cmdstree), 1, 1);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"altitude PID",
		{"P",			"I",			"D"},
		{"pid altitude p",	"pid altitude i",	"pid altitude d"},
		cmdstree), 1, 2);

	tabs["filters"]->add_group(new float_settings_group(nullptr,
		"Complimentary filters",
		{"attitude",		"yaw",		"climb rate", 		"altitude"},
		{"compl attitude",	"compl yaw",	"compl climbrate",	"compl altitude"},
		cmdstree), 3, 0, 2, 1); 
	tabs["filters"]->add_group(new float_settings_group(nullptr,
		"Low-pass filters",
		{"gyroscope",	"accelerometer",	"d-term",	"climb rate",	"acceleration",	"altitude"},
		{"lpf gyro",	"lpf accel",		"lpf d",	"lpf climb",	"lpf vaccel",	"lpf altitude"},
		cmdstree), 3, 1, 2, 1);

	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"attitude offset",
		{"roll",	"pitch",	"yaw"},
		{"adj roll",	"adj pitch",	"adj yaw"},
		cmdstree), 0, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"motor scale",
		{"roll",		"pitch"},
		{"adj rollthrust", 	"adj pitchthrust"},
		cmdstree), 0, 1);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"accelerometer offset",
		{"X",		"Y",		"Z"},
		{"adj acc x",	"adj acc y",	"adj acc z"},
		cmdstree), 1, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"gyroscope offset",
		{"X",		"Y",		"Z"},
		{"adj gyro x",	"adj gyro y",	"adj gyro z"},
		cmdstree), 1, 1);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"magnetometer offsets",
		{"X",		"Y",		"Z",		"declination"},
		{"adj mag x0", 	"adj mag y0",	"adj mag z0", 	"adj mag decl"},
		cmdstree), 2, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"magnetometer scale",
		{"X",			"Y",			"Z"},
		{"adj mag xscale", 	"adj mag yscale",	"adj mag zscale"},
		cmdstree), 2, 1);

	tabs["control"]->add_group(new float_settings_group(nullptr,
		"control maximums",
		{"thrust",	"roll angle", 	"pitch angle",	"acceleration",	"altitude"},
		{"ctrl thrust",	"ctrl roll",	"ctrl pitch",	"ctrl accel",	"ctrl altmax"},
		cmdstree), 3, 2, 2, 1);
	
	tabs["control"]->add_group(new float_settings_group(nullptr,
		"control rates",
		{"roll",	"pitch",	"yaw position",	"yaw",		"climb"},
		{"ctrl sroll",	"ctrl spitch",	"ctrl yaw",	"ctrl syaw",	"ctrl climbrate"},
		cmdstree), 3, 3, 2, 1);

	
	settings_group *info = new settings_group(nullptr, "Info");
	info->add_setting_pair(
		new button_setting(nullptr, "IMU", info_imu_click_handler, this),
		new button_setting(nullptr, "PID", info_pid_click_handler, this)
	);
	info->add_setting_pair(
		new button_setting(nullptr, "Values", info_values_click_handler, this),
		new button_setting(nullptr, "Magnetometer", info_mag_click_handler, this)
	);
	info->add_setting_pair(
		new button_setting(nullptr, "Barometer", info_bar_click_handler, this),
		new button_setting(nullptr, "GNSS", info_gnss_click_handler, this)
	);
	info->add_setting_pair(
		new button_setting(nullptr, "Devices status", info_devices_click_handler, this),
		new button_setting(nullptr, "Control", info_control_click_handler, this)
	);
	info->add_setting_pair(
		new button_setting(nullptr, "Filters", info_filter_click_handler, this),
		new button_setting(nullptr, "VTX", info_vtx_click_handler, this)
	);
	
	tabs["info"]->add_group(info, 0, 0, 1, 1);

	settings_group *log_write = new settings_group(nullptr, "Log write");
	settings_group *log_read = new settings_group(nullptr, "Log read");

	log_write->add_setting(new uint_setting(nullptr,
		"records count"));
	log_write->add_setting_pair(
		new button_setting(nullptr, "start log", log_start_click_handler, this),
		new button_setting(nullptr, "stop log", log_stop_click_handler, this));

	log_read->add_setting(new uint_setting(nullptr, "from"));
	log_read->add_setting(new uint_setting(nullptr, "to"));
	log_read->add_setting(new button_setting(nullptr, "read log", log_read_click_handler, this),
		false);
	log_read->add_setting(new button_setting(nullptr, "load log", log_load_click_handler, this),
		false);

	tabs["log"]->add_group(log_write, 0, 0, 1, 1);
	tabs["log"]->add_group(log_read, 0, 1, 1, 1);

	tab->addTab(tabs["pid"], "PID");
	tab->addTab(tabs["filters"], "Filters");
	tab->addTab(tabs["adjustments"], "Adjustments");
	tab->addTab(tabs["control"], "Control");
	tab->addTab(tabs["log"], "Log");
	tab->addTab(tabs["info"], "Info");

	settings["open"] = new button_setting(nullptr, "open config", open_click_handler, this);
	settings["save"] = new button_setting(nullptr, "save config", save_click_handler, this);
	settings["connect"] = new button_setting(nullptr, "connect to UAV", connect_click_handler, this);
	settings["flash"] = new button_setting(nullptr, "write to flash", flash_click_handler, this);

	connect(term->get_line(), &QLineEdit::returnPressed, this, &main_widget::return_pressed);

	timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, &main_widget::timer_handler);
	timer->start(1);

	grid->addWidget(tab, 0, 0, 5, 4);
	grid->addWidget(settings["open"]->get_field(), 5, 0);
	grid->addWidget(settings["save"]->get_field(), 5, 1);
	grid->addWidget(settings["connect"]->get_field(), 5, 2);
	grid->addWidget(settings["flash"]->get_field(), 5, 3);
	grid->addWidget(term, 0, 4, 6, 2);

	setWindowTitle(tr("Settings"));
}

main_widget::~main_widget()
{
	for (auto it = begin(tabs); it != end(tabs); ++it)
		delete it->second;
	
	for (auto it = begin(settings); it != end(settings); ++it)
		delete it->second;

	delete term;
	delete tab;
	delete grid;
	delete timer;
	
	delete cmdstree;
}

void main_widget::write_to_terminal(string s)
{
	term->add_output(s);
}

void main_widget::string_to_conf(const string &s)
{
	stringstream ss(s);
	string line;

	while (getline(ss, line, '\n')) {
		stringstream ss(line);
		vector<string> toks;
		string tok;

		while (getline(ss, tok, ' '))
			toks.push_back(tok);

		commands_tree *tr;

		tr = cmdstree;
		for (auto it = begin(toks); it != (end(toks) - 1); ++it)
			tr = tr->get_child(*it);

		if (tr->get_setting() != nullptr)
			tr->get_setting()->set_value(*(end(toks) - 1));
	}
}

string main_widget::conf_to_string()
{
	string conf;

	conf = string("");

	for (auto t = begin(tabs); t != end(tabs); ++t) {
		map<string, settings_group *> &groups
			= t->second->get_groups();

		for (auto g = begin(groups); g != end(groups); ++g) {
			map<string, setting *> &sts
				= g->second->get_settings();

			for (auto s = begin(sts); s != end(sts); ++s) {
				if (s->second->get_command().empty())
					continue;

				conf += s->second->get_command()
					+ string(" ")
					+ s->second->get_value()
					+ string("\n");
			}
		}
	}

	conf += "flash write" + string("\n");

	return conf;
}

void main_widget::send_uav_conf_cmd(string cmd)
{
	sendcmd(lsfd, &rsi, cmd.c_str(), conffunc,
		write_term, (void *) term);
}

void main_widget::send_uav_info_cmd(string cmd)
{
	sendcmd(lsfd, &rsi, cmd.c_str(), infofunc,
		write_term, (void *) term);
}

void main_widget::send_uav_get_cmd(string cmd, string *out)
{
	sendcmd(lsfd, &rsi, cmd.c_str(), getfunc,
		write_conf, out);
}

void main_widget::get_uav_log(string &s)
{
	char *output;
	size_t outsize;

	getlog(lsfd, &rsi, 
		stoi(tabs["log"]->get_group("Log read")
		 	->get_setting("from")->get_value()),
		stoi(tabs["log"]->get_group("Log read")
		 	->get_setting("to")->get_value()),
		&output, &outsize);

	if (outsize == 0)
		return;

	s = string(output);
	
	free(output);
}

void main_widget::keyPressEvent(QKeyEvent *event)
{
	if (event->key() == Qt::Key_M)
		info_imu_click_handler(this);
	else if (event->key() == Qt::Key_P)
		info_pid_click_handler(this);
	else if (event->key() == Qt::Key_V)
		info_values_click_handler(this);
	else if (event->key() == Qt::Key_H)
		info_mag_click_handler(this);
	else if (event->key() == Qt::Key_B)
		info_bar_click_handler(this);
	else if (event->key() == Qt::Key_G)
		info_gnss_click_handler(this);
	else if (event->key() == Qt::Key_D)
		info_devices_click_handler(this);
	else if (event->key() == Qt::Key_T)
		info_control_click_handler(this);
	else if (event->key() == Qt::Key_F)
		info_filter_click_handler(this);
	else if (event->key() == Qt::Key_I)
		info_vtx_click_handler(this);
}

void main_widget::return_pressed()
{
	string cmd;

	cmd = term->get_line()->text().toStdString();

	sendcmd(lsfd, &rsi, cmd.c_str(), infofunc,
		write_term, (void *) term);

	term->get_line()->setText("");
}

void main_widget::timer_handler()
{
	char buf[BUFSZ];

	if (!catchuavout)
		return;

	if (recvoutput(lsfd, &rsi, buf) != 0)
		return;

	term->add_output(string(buf));
	QCoreApplication::processEvents();
}
