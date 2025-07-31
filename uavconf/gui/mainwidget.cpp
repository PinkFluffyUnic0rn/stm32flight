#include <string>
#include <vector>
#include <set>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <QtWidgets>
#include "mainwidget.h"

extern "C" {
#include "../api/uavconf.h"
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

	mw->write_to_terminal(string("Connected to UAV\n"));

	mw->string_to_conf(conf);
	
	mw->start_catch_uav_out();
}

void send_click_handler(void *arg)
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

void flash_click_handler(void *arg)
{
	map<string, settings_tab *> tabs;
	main_widget *mw;
	stringstream ss;
	string line;
	string conf;
	
	mw = (main_widget *) arg;
	
	tabs = mw->get_tabs();

	mw->stop_catch_uav_out();
	
	mw->send_uav_conf_cmd("flash write " + tabs["flash"]
		->get_group("Write")
		->get_setting("slot")
		->get_value() + "\n");

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

void lt_direct_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor lt d");
}

void lt_reverse_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor lt r");
}

void lt_save_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor lt s");
}

void lb_direct_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor lb d");
}

void lb_reverse_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor lb r");
}

void lb_save_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor lb s");
}

void rt_direct_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor rt d");
}

void rt_reverse_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor rt r");
}

void rt_save_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor rt s");
}

void rb_direct_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor rb d");
}

void rb_reverse_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor rb r");
}

void rb_save_click_handler(void *arg)
{
	((main_widget *) arg)->send_uav_info_cmd("motor rb s");
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

	if (t.find('.') != std::string::npos) {
		t.erase(t.find_last_not_of("0\r\n") + 1);

		if (t.back() == '.')
			t.pop_back();
	}

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

	if (t.find('.') != std::string::npos) {
		t.erase(t.find_last_not_of("0\r\n") + 1);

		if (t.back() == '.')
			t.pop_back();
	}

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
	commands_tree *cmdstree, vector<string> modes, string init)
		: setting(parent, n, c, cmdstree)
{
	box = new QComboBox();

	for (auto it = begin(modes); it != end(modes); ++it)
		box->addItem((*it).c_str());


	if ((def_idx = box->findText(QString::fromStdString(init))) < 0)
		def_idx = 0;

	box->setCurrentIndex(def_idx);
}

mode_setting::~mode_setting()
{
	delete box;
}

string mode_setting::get_value() const
{
	return box->currentText().toStdString();
}

void mode_setting::set_value(const string &s)
{
	int idx;

	if ((idx = box->findText(QString::fromStdString(s))) < 0)
		idx = def_idx;

	box->setCurrentIndex(idx);
}

void partial_send_click_handler(void *arg)
{
	stringstream ss;
	string line;
	settings_group *g;
	string conf;

	g = (settings_group *) arg;

	map<string, setting *> &sts = g->get_settings();

	for (auto s = begin(sts); s != end(sts); ++s) {
		if (s->second->get_command().empty())
			continue;
	

		conf += s->second->get_command()
			+ string(" ")
			+ s->second->get_value()
			+ string("\n");
	}
		
	if (g->get_main_widget() == NULL)
		return;

	g->get_main_widget()->stop_catch_uav_out();
	
	ss = stringstream(conf);

	while (getline(ss, line, '\n')) {
		g->get_main_widget()->send_uav_conf_cmd(line);
		QCoreApplication::processEvents();
	}
	
	g->get_main_widget()->start_catch_uav_out();
}

settings_group::settings_group(QWidget *parent, string n,
	bool nsb, main_widget *mw)
		: QWidget(parent), name(n), _main_widget(mw),
			has_send_button(nsb), layout_last(0)
{
	group = new QGroupBox(QString::fromStdString(name),
		dynamic_cast<QWidget *>(this));

	layout = new QGridLayout(group);

	if (nsb) {
		settings["send"] = new button_setting(nullptr, "send",
			partial_send_click_handler, this);

		layout->addWidget(settings["send"]->get_field(),
			layout_last++, 0, 1, 2);
	}
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
	QWidget *w;
	
	settings[s->get_name()] = s;

	w = nullptr;
	if (has_send_button) {
		--layout_last;
		w = layout->itemAtPosition(layout_last, 0)->widget();
		layout->takeAt(layout->indexOf(w));
	}

	if (addlabel) {
		layout->addWidget(s->get_label(), layout_last, 0);
		layout->addWidget(s->get_field(), layout_last, 1);
	}
	else
		layout->addWidget(s->get_field(), layout_last, 0, 1, 2);

	++layout_last;

	if (w != nullptr) {
		layout->addWidget(w, layout_last, 0, 1, 2);
		++layout_last;
	}
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
		commands_tree *cmdstree, bool nsb,
		main_widget *mw)
		: settings_group(parent, name, nsb, mw)
{
	size_t i;

	for (i = 0; i < s.size(); ++i) {
		add_setting(new float_setting(nullptr,
			s[i], c[i], cmdstree));
	}
}

uint_settings_group::uint_settings_group(QWidget *parent,
	string name, vector<string> s, vector<string> c,
		commands_tree *cmdstree, bool nsb,
		main_widget *mw)
		: settings_group(parent, name, nsb, mw)
{
	size_t i;

	for (i = 0; i < s.size(); ++i) {
		add_setting(new uint_setting(nullptr,
			s[i], c[i], cmdstree));
	}
}

mode_settings_group::mode_settings_group(QWidget *parent,
	string name, vector<string> s, vector<string> c,
		commands_tree *cmdstree, vector<string> modes,
		string init, bool nsb, main_widget *mw)
		: settings_group(parent, name, nsb, mw)
{
	size_t i;

	for (i = 0; i < s.size(); ++i) {
		add_setting(new mode_setting(nullptr,
			s[i], c[i], cmdstree, modes, init));
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

void settings_tab::add_widget(QWidget *w,
	int r, int c, int rs, int cs)
{
	grid->addWidget(w, r, c, rs, cs);
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

void main_widget::record_size_item_changed(int idx)
{
	int size;
	int i;

	(void) idx;

	size = stoi(tabs["log"]->get_group("Log config")
		->get_setting("Record size")->get_value());

	for (i = 0; i < 32; ++i) {
		string g;

		if (i >= 0 && i < 11)
			g = "Record fields 0-10";
		else if (i >= 11 && i < 22)
			g = "Record fields 11-21";
		else
			g = "Record fields 22-31";

		tabs["log"]->get_group(g)
			->get_setting(std::to_string(i))
			->get_field()->setEnabled((i < size));
	}
}

void main_widget::record_field_item_changed(int idx)
{
	setting *st[32];
	set<string> ss;
	set<string> ssp;
	int fieldidx;
	int i;
	
	(void) idx;

	fieldidx = 0;

	for (i = 0; i < 32; ++i) {
		string g;

		if (i >= 0 && i < 11)
			g = "Record fields 0-10";
		else if (i >= 11 && i < 22)
			g = "Record fields 11-21";
		else
			g = "Record fields 22-31";

		st[i] = tabs["log"]->get_group(g)
			->get_setting(std::to_string(i));
	}

	for (i = 0; i < 32; ++i) {
		if (sender() == st[i]->get_field()) {
			fieldidx = i;
			break;
		}
	}
	
	for (i = 0; i < 32; ++i) {
		if (i != fieldidx
				&& st[i]->get_value() == st[fieldidx]->get_value()
				&& st[i]->get_value() != "none") {
			st[i]->set_value("none");
		}
	}
}

void main_widget::motor_mapping_item_changed(int idx)
{
	setting *st[4];
	set<string> ss;
	int motoridx;
	int i;
	
	(void) idx;

	motoridx = 0;

	st[0] = this->tabs["motors"]->get_group("left-top")->get_setting("output");
	st[1] = this->tabs["motors"]->get_group("left-bottom")->get_setting("output");
	st[2] = this->tabs["motors"]->get_group("right-top")->get_setting("output");
	st[3] = this->tabs["motors"]->get_group("right-bottom")->get_setting("output");

	for (i = 0; i < 4; ++i) {
		if (sender() == st[i]->get_field()) {
			motoridx = i;
			break;
		}
	}

	for (i = 0; i < 4; ++i)
		ss.insert(st[i]->get_value());

	for (i = 0; i < 4; ++i) {
		if (i != motoridx
				&& st[i]->get_value() == st[motoridx]->get_value()) {
			set<string> ssd;
			set<string> ssp;

			ssp = set<string>({"0", "1", "2", "3"});

			set_difference(ssp.begin(), ssp.end(),
				ss.begin(), ss.end(), inserter(ssd, ssd.begin()));
	
			st[i]->set_value(*(ssd.begin()));
		}
	}
}

main_widget::main_widget(QWidget *parent)
	: QWidget(parent), catchuavout(true)
{
	int i;

	grid = new QGridLayout(this);
	tab = new QTabWidget;
	term = new terminal();
	
	cmdstree = new commands_tree;

	initsocks(&lsfd, &rsi);

	tabs["pid"] = new settings_tab;
	tabs["adjustments"] = new settings_tab;
	tabs["filters"] = new settings_tab;
	tabs["control"] = new settings_tab;
	tabs["flash"] = new settings_tab;
	tabs["log"] = new settings_tab;
	tabs["info"] = new settings_tab;
	tabs["devices"] = new settings_tab;
	tabs["motors"] = new settings_tab;
	
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"Rate PID", 
		{"P",		"I", 		"D"},
		{"pid stilt p",	"pid stilt i",	"pid stilt d"},
		cmdstree, true, this), 0, 0);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"Angle PID",
		{"P",		"I",		"D"},
		{"pid tilt p",	"pid tilt i",	"pid tilt d"},
		cmdstree, true, this), 0, 1);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"Yaw PID",
		{"P",		"I",		"D"},
		{"pid syaw p",	"pid syaw i",	"pid syaw d"},
		cmdstree, true, this), 0, 2);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"Yaw position PID",
		{"P",		"I",		"D"},
		{"pid yaw p",	"pid yaw i",	"pid yaw d"},
		cmdstree, true, this), 0, 3);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"Throttle PID",
		{"P",			"I",			"D"},
		{"pid throttle p", 	"pid throttle i",	"pid throttle d"},
		cmdstree, true, this), 1, 0);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"Climb rate PID",
		{"P",			"I",			"D"},
		{"pid climbrate p",	"pid climbrate i",	"pid climbrate d"},
		cmdstree, true, this), 1, 1);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"Altitude PID",
		{"P",			"I",			"D"},
		{"pid altitude p",	"pid altitude i",	"pid altitude d"},
		cmdstree, true, this), 1, 2);

	tabs["filters"]->add_group(new float_settings_group(nullptr,
		"Complimentary filters",
		{"attitude",		"yaw",		"climb rate", 		"altitude"},
		{"compl attitude",	"compl yaw",	"compl climbrate",	"compl altitude"},
		cmdstree, true, this), 3, 0, 2, 1); 
	tabs["filters"]->add_group(new float_settings_group(nullptr,
		"Low-pass filters",
		{"gyroscope",	"accelerometer",	"d-term",	"climb rate",	"acceleration",	"altitude"},
		{"lpf gyro",	"lpf accel",		"lpf d",	"lpf climb",	"lpf vaccel",	"lpf altitude"},
		cmdstree, true, this), 3, 1, 2, 1);

	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"Accelerometer offset",
		{"X",		"Y",		"Z"},
		{"adj acc x",	"adj acc y",	"adj acc z"},
		cmdstree, true, this), 0, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"Gyroscope offset",
		{"X",		"Y",		"Z"},
		{"adj gyro x",	"adj gyro y",	"adj gyro z"},
		cmdstree, true, this), 0, 1);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"Attitude offset",
		{"roll",	"pitch",	"yaw"},
		{"adj roll",	"adj pitch",	"adj yaw"},
		cmdstree, true, this), 0, 2);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"Magnetometer offsets",
		{"X",		"Y",		"Z",		"declination"},
		{"adj mag x0", 	"adj mag y0",	"adj mag z0", 	"adj mag decl"},
		cmdstree, true, this), 2, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"Magnetometer scale",
		{"X",			"Y",			"Z"},
		{"adj mag xscale", 	"adj mag yscale",	"adj mag zscale"},
		cmdstree, true, this), 2, 1);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"Motor scale",
		{"roll",		"pitch"},
		{"adj rollthrust", 	"adj pitchthrust"},
		cmdstree, true, this), 2, 2);
	
	tabs["control"]->add_group(new float_settings_group(nullptr,
		"Control maximums",
		{"thrust",	"roll angle", 	"pitch angle",	"acceleration",	"altitude"},
		{"ctrl thrust",	"ctrl roll",	"ctrl pitch",	"ctrl accel",	"ctrl altmax"},
		cmdstree, true, this), 3, 2, 2, 1);
	
	tabs["control"]->add_group(new float_settings_group(nullptr,
		"Control rates",
		{"roll",	"pitch",	"yaw position",	"yaw",		"climb"},
		{"ctrl sroll",	"ctrl spitch",	"ctrl yaw",	"ctrl syaw",	"ctrl climbrate"},
		cmdstree, true, this), 3, 3, 2, 1);

	settings_group *flash = new settings_group(nullptr, "Write");

	flash->add_setting(new uint_setting(nullptr, "slot"));
	flash->add_setting(new button_setting(nullptr, "write to flash",
		flash_click_handler, this), false);

	tabs["flash"]->add_group(flash, 0, 0, 1, 1);

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
	settings_group *log_config = new settings_group(nullptr, "Log config", true, this);

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

	log_config->add_setting(new mode_setting(nullptr, "Record size",
		"log record size", cmdstree, {"1", "2", "4", "8", "16", "32"}, "8"));

	connect(log_config->get_setting("Record size")->get_field(),
		SIGNAL(currentIndexChanged(int)), this,
		SLOT(record_size_item_changed(int)));

	log_config->add_setting(new uint_setting(nullptr, "Log frequency",
		"log freq", cmdstree));

	tabs["log"]->add_group(log_config, 0, 0, 1, 2);
	tabs["log"]->add_group(log_write, 1, 0, 1, 2);
	tabs["log"]->add_group(log_read, 2, 0, 1, 2);

	tabs["log"]->add_group(new mode_settings_group(nullptr,
		"Record fields 0-10",
		{
			"0", "1", "2", "3", "4", "5",
			"6", "7", "8", "9", "10"
		},
		{
			"log record 0", "log record 1", "log record 2",
			"log record 3", "log record 4", "log record 5",
			"log record 6", "log record 7", "log record 8",
			"log record 9", "log record 10"
		},
		cmdstree, {
			"acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y",
			"gyro_z", "mag_x", "mag_y", "mag_z", "bar_temp",
			"bar_alt", "roll", "pitch", "yaw", "climbrate",
			"alt", "lt", "lb", "rb", "rt", "bat", "cur",
			"ch0", "ch1", "ch2", "ch3", "ch4", "ch5", "ch6",
			"ch7", "ch8", "ch9", "ch10", "ch11", "ch12",
			"ch13", "ch14", "ch15", "none"
		}, "none", true, this), 0, 2, 3, 1);

	tabs["log"]->add_group(new mode_settings_group(nullptr,
		"Record fields 11-21",
		{
			"11", "12", "13", "14", "15", "16",
			"17", "18", "19", "20", "21"
		},
		{
			"log record 11", "log record 12", "log record 13",
			"log record 14", "log record 15", "log record 16",
			"log record 17", "log record 18", "log record 19",
			"log record 20", "log record 21"
		},
		cmdstree, {
			"acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y",
			"gyro_z", "mag_x", "mag_y", "mag_z", "bar_temp",
			"bar_alt", "roll", "pitch", "yaw", "climbrate",
			"alt", "lt", "lb", "rb", "rt", "bat", "cur",
			"ch0", "ch1", "ch2", "ch3", "ch4", "ch5", "ch6",
			"ch7", "ch8", "ch9", "ch10", "ch11", "ch12",
			"ch13", "ch14", "ch15", "none"
		}, "none", true, this), 0, 3, 3, 1);

	tabs["log"]->add_group(new mode_settings_group(nullptr,
		"Record fields 22-31",
		{
			"22", "23", "24", "25", "26", "27",
			"28", "29", "30", "31"
		},
		{
			"log record 22", "log record 23", "log record 24",
			"log record 25", "log record 26", "log record 27",
			"log record 28", "log record 29", "log record 30",
			"log record 31"
		},
		cmdstree, {
			"acc_x", "acc_y", "acc_z", "gyro_x", "gyro_y",
			"gyro_z", "mag_x", "mag_y", "mag_z", "bar_temp",
			"bar_alt", "roll", "pitch", "yaw", "climbrate",
			"alt", "lt", "lb", "rb", "rt", "bat", "cur",
			"ch0", "ch1", "ch2", "ch3", "ch4", "ch5", "ch6",
			"ch7", "ch8", "ch9", "ch10", "ch11", "ch12",
			"ch13", "ch14", "ch15", "none"
		}, "none", true, this), 0, 4, 3, 1);


	for (i = 0; i < 32; ++i) {
		string g;

		if (i >= 0 && i < 11)
			g = "Record fields 0-10";
		else if (i >= 11 && i < 22)
			g = "Record fields 11-21";
		else
			g = "Record fields 22-31";

		connect(tabs["log"]->get_group(g)
			->get_setting(std::to_string(i))->get_field(),
			SIGNAL(currentIndexChanged(int)), this,
			SLOT(record_field_item_changed(int)));
	}

	connect(log_config->get_setting("Record size")->get_field(),
		SIGNAL(currentIndexChanged(int)), this,
		SLOT(record_size_item_changed(int)));



	settings_group *irc = new settings_group(nullptr, "IRC",
		true, this);
	irc->add_setting(new mode_setting(nullptr, "Power", "irc power",
		cmdstree, {"25", "100", "200", "400", "600"}));
	irc->add_setting(new mode_setting(nullptr, "Frequency",
		"irc frequency", cmdstree, {"5865", "5845", "5825",
		"5805", "5785", "5765", "5745", "5725", "5733", "5752",
		"5771", "5790", "5809", "5828", "5847", "5866", "5705",
		"5685", "5665", "5645", "5885", "5905", "5925", "5945",
		"5740", "5760", "5780", "5800", "5820", "5840", "5860",
		"5880", "5658", "5695", "5732", "5769", "5806", "5843",
		"5917"}));

	tabs["devices"]->add_group(irc, 0, 0, 1, 1);

	settings_group *lt = new settings_group(nullptr,
		"left-top");
	lt->add_setting(new mode_setting(nullptr, "output",
		"motor lt", cmdstree, {"0", "1", "2", "3"}, "0"));
	lt->add_setting_pair(
		new button_setting(nullptr, "direct", lt_direct_click_handler, this),
		new button_setting(nullptr, "reverse", lt_reverse_click_handler, this)
	);
	lt->add_setting(
		new button_setting(nullptr, "save direction", lt_save_click_handler, this),
		false
	);

	tabs["motors"]->add_group(lt, 0, 0, 1, 1);

	settings_group *lb = new settings_group(nullptr,
		"left-bottom");
	lb->add_setting(new mode_setting(nullptr, "output",
		"motor lb", cmdstree, {"0", "1", "2", "3"}, "1"));
	lb->add_setting_pair(
		new button_setting(nullptr, "direct", lb_direct_click_handler, this),
		new button_setting(nullptr, "reverse", lb_reverse_click_handler, this)
	);
	lb->add_setting(
		new button_setting(nullptr, "save direction", lb_save_click_handler, this),
		false
	);

	tabs["motors"]->add_group(lb, 1, 0, 1, 1);

	settings_group *rt = new settings_group(nullptr,
		"right-top");
	rt->add_setting(new mode_setting(nullptr, "output",
		"motor rt", cmdstree, {"0", "1", "2", "3"}, "2"));
	rt->add_setting_pair(
		new button_setting(nullptr, "direct", rt_direct_click_handler, this),
		new button_setting(nullptr, "reverse", rt_reverse_click_handler, this)
	);
	rt->add_setting(
		new button_setting(nullptr, "save direction", rt_save_click_handler, this),
		false
	);

	tabs["motors"]->add_group(rt, 0, 1, 1, 1);

	settings_group *rb = new settings_group(nullptr,
		"right-bottom");
	rb->add_setting(new mode_setting(nullptr, "output",
		"motor rb", cmdstree, {"0", "1", "2", "3"}, "3"));
	rb->add_setting_pair(
		new button_setting(nullptr, "direct", rb_direct_click_handler, this),
		new button_setting(nullptr, "reverse", rb_reverse_click_handler, this)
	);
	rb->add_setting(
		new button_setting(nullptr, "save direction", rb_save_click_handler, this),
		false
	);
	
	tabs["motors"]->add_group(rb, 1, 1, 1, 1);

	connect(lt->get_setting("output")->get_field(),
		SIGNAL(currentIndexChanged(int)), this,
		SLOT(motor_mapping_item_changed(int)));
	connect(lb->get_setting("output")->get_field(),
		SIGNAL(currentIndexChanged(int)), this,
		SLOT(motor_mapping_item_changed(int)));
	connect(rt->get_setting("output")->get_field(),
		SIGNAL(currentIndexChanged(int)), this,
		SLOT(motor_mapping_item_changed(int)));
	connect(rb->get_setting("output")->get_field(),
		SIGNAL(currentIndexChanged(int)), this,
		SLOT(motor_mapping_item_changed(int)));

	tab->addTab(tabs["pid"], "PID");
	tab->addTab(tabs["filters"], "Filters");
	tab->addTab(tabs["adjustments"], "Adjustments");
	tab->addTab(tabs["control"], "Control");
	tab->addTab(tabs["flash"], "Flash");
	tab->addTab(tabs["log"], "Log");
	tab->addTab(tabs["info"], "Info");
	tab->addTab(tabs["devices"], "Devices");
	tab->addTab(tabs["motors"], "Motors");

	settings["open"] = new button_setting(nullptr, "open config", open_click_handler, this);
	settings["save"] = new button_setting(nullptr, "save config", save_click_handler, this);
	settings["connect"] = new button_setting(nullptr, "get settings", connect_click_handler, this);
	settings["send"] = new button_setting(nullptr, "set settings", send_click_handler, this);

	connect(term->get_line(), &QLineEdit::returnPressed, this, &main_widget::return_pressed);

	timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, &main_widget::timer_handler);
	timer->start(1);

	grid->addWidget(tab, 0, 0, 5, 4);
	grid->addWidget(settings["open"]->get_field(), 5, 0);
	grid->addWidget(settings["save"]->get_field(), 5, 1);
	grid->addWidget(settings["connect"]->get_field(), 5, 2);
	grid->addWidget(settings["send"]->get_field(), 5, 3);
	grid->addWidget(term, 0, 4, 6, 2);

	record_size_item_changed(0);

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
		stringstream ss;
		vector<string> toks;
		commands_tree *tr;
		string tok;

		while (line.back() == '\n' || line.back() == '\r')
			line.pop_back();

		ss = stringstream(line);
		
		while (getline(ss, tok, ' '))
			toks.push_back(tok);

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
