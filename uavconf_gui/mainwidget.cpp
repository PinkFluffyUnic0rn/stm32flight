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

	validator = new QRegExpValidator(	
		QRegExp("\\d{0,}"),0);

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

button_setting::button_setting(QWidget *parent, string n, string c,
	commands_tree *cmdstree) : setting(parent, n, c, cmdstree)
{
	button = new QPushButton(QString::fromStdString(n));
}

button_setting::~button_setting()
{
	delete button;
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

void terminal::add_output(string s)
{
	edit->moveCursor(QTextCursor::End);
	edit->insertPlainText(QString::fromStdString(s));
	edit->moveCursor(QTextCursor::End);
}

void main_widget::conf_to_string()
{
	conf = string("");

	for (auto t = begin(tabs); t != end(tabs); ++t) {
		map<string, settings_group *> &groups
			= t->second->get_groups();

		for (auto g = begin(groups); g != end(groups); ++g) {
			map<string, setting *> &settings
				= g->second->get_settings();

			for (auto s = begin(settings); s != end(settings); ++s) {
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

		tr = &cmdstree;
		for (auto it = begin(toks); it != (end(toks) - 1); ++it) {
			tr = tr->get_child(*it);
		}

		if (tr->get_setting() != nullptr)
			tr->get_setting()->set_value(*(end(toks) - 1));
	}
}

void main_widget::open_click_handler()
{
	QString name;
	ifstream infile;
	stringstream buf;

	name = QFileDialog::getOpenFileName(this, tr("Open file"),
		"", tr("Text files (*.txt)"));

	infile.open(name.toStdString());	

	buf << infile.rdbuf();

	string_to_conf(buf.str());
}

void main_widget::save_click_handler()
{
	QString name;
	ofstream outfile;

	name = QFileDialog::getSaveFileName(this, tr("Save file"), "",
		tr("Text files (*.txt)"));

	conf_to_string();

	outfile.open(name.toStdString());

	outfile << conf;

	outfile.close();
}

void write_conf(void *c, const char *s)
{
	string *conf;

	conf = (string *) c;

	*conf += string(s);
}

void main_widget::connect_click_handler()
{
	conf = "";
	
	catchuavout = false;
	
	term->add_output(string("loading values..."));

	conf = string("");

	for (auto t = begin(tabs); t != end(tabs); ++t) {
		map<string, settings_group *> &groups
			= t->second->get_groups();

		for (auto g = begin(groups); g != end(groups); ++g) {
			map<string, setting *> &settings
				= g->second->get_settings();

			for (auto s = begin(settings); s != end(settings); ++s) {
				if (s->second->get_command().empty())
					continue;

				sendcmd(lsfd, &rsi,
					("get " + s->second->get_command()).c_str(),
					getfunc, write_conf, (void *) &conf);
			
				term->add_output("get " + s->second->get_command() + "\n");
				
				QCoreApplication::processEvents();
			}
		}
	}

	term->add_output(string("Connected to UAV, values loaded"));

	string_to_conf(conf);
	
	catchuavout = true;
}

void write_term(void *t, const char *s)
{
	terminal *term;

	term = (terminal *) t;

	term->add_output(string(s) + "\n");
	QCoreApplication::processEvents();
}

void main_widget::return_pressed()
{
	string cmd;

	cmd = term->get_line()->text().toStdString();

	sendcmd(lsfd, &rsi, cmd.c_str(), infofunc,
		write_term, (void *) term);

	term->get_line()->setText("");
}

void main_widget::flash_click_handler()
{
	stringstream ss;
	string line;
	
	catchuavout = false;
	
	conf_to_string();

	ss = stringstream(conf);

	while (getline(ss, line, '\n')) {
		sendcmd(lsfd, &rsi, line.c_str(), conffunc,
			write_term, (void *) term);
	
		QCoreApplication::processEvents();
	}
	
	catchuavout = true;
}

void main_widget::start_log_click_handler()
{
	sendcmd(lsfd, &rsi,
		("log set " + tabs["log"]->get_group("Log write")
		 	->get_setting("records count")
			->get_value() + "\n").c_str(),
		infofunc, write_term, (void *) term);
}

void main_widget::stop_log_click_handler()
{
	sendcmd(lsfd, &rsi, "log set 0\n", infofunc,
		write_term, (void *) term);
}

void main_widget::read_log_click_handler()
{
	sendcmd(lsfd, &rsi,
		("log rget " + tabs["log"]->get_group("Log read")
		 	->get_setting("from")
			->get_value() + " "
		 + tabs["log"]->get_group("Log read")
		 	->get_setting("to")
			->get_value() + "\n").c_str(),
		infofunc, write_term, (void *) term);
}

void main_widget::load_log_click_handler()
{
	char *output;
	size_t outsize;

	catchuavout = false;
	getlog(lsfd, &rsi, 
		stoi(tabs["log"]->get_group("Log read")
		 	->get_setting("from")->get_value()),
		stoi(tabs["log"]->get_group("Log read")
		 	->get_setting("to")->get_value()),
		&output, &outsize);

	if (outsize == 0)
		return;

	QString name;
	ofstream outfile;

	name = QFileDialog::getSaveFileName(this, tr("Save file"), "",
		tr("Text files (*.txt)"));

	outfile.open(name.toStdString());

	outfile << string(output);

	outfile.close();

	catchuavout = true;
}

commands_tree::commands_tree(const string &n) : cmdsetting(nullptr)
{
	name = n;
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

#define BUFSZ 1024

void main_widget::timer_handler()
{
	fd_set rfds;
	struct timeval timeout;

	if (!catchuavout)
		return;

	FD_ZERO(&rfds);
	FD_SET(lsfd, &rfds);

	timeout.tv_sec = 0;
	timeout.tv_usec = 0;

	if (select(lsfd + 1, &rfds, NULL, NULL, &timeout) <= 0)
		return;

	if (FD_ISSET(lsfd, &rfds)) {
		char buf[BUFSZ + 1];
		socklen_t rsis;
		int rsz;

		rsis = sizeof(rsi);
		if ((rsz = recvfrom(lsfd, buf, BUFSZ, 0,
			(struct sockaddr *) &rsi, &rsis)) <= 0) {
			return;
		}
			
		buf[rsz] = '\0';
		
		term->add_output(string(buf));
		QCoreApplication::processEvents();

	}
}

main_widget::main_widget(QWidget *parent)
	: QWidget(parent), catchuavout(true)
{
	grid = new QGridLayout(this);
	tab = new QTabWidget;

	tabs["pid"] = new settings_tab;
	tabs["adjustments"] = new settings_tab;
	tabs["filters"] = new settings_tab;
	tabs["control"] = new settings_tab;
	tabs["log"] = new settings_tab;
	
	term = new terminal();

	initsocks(&lsfd, &rsi);

	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"rate PID", 
		{"P",		"I", 		"D"},
		{"pid stilt p",	"pid stilt i",	"pid stilt d"},
		&cmdstree), 0, 0);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"angle PID",
		{"P",		"I",		"D"},
		{"pid tilt p",	"pid tilt i",	"pid tilt d"},
		&cmdstree), 0, 1);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"yaw PID",
		{"P",		"I",		"D"},
		{"pid syaw p",	"pid syaw i",	"pid syaw d"},
		&cmdstree), 0, 2);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"yaw position PID",
		{"P",		"I",		"D"},
		{"pid yaw p",	"pid yaw i",	"pid yaw d"},
		&cmdstree), 0, 3);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"throttle PID",
		{"P",			"I",			"D"},
		{"pid throttle p", 	"pid throttle i",	"pid throttle d"},
		&cmdstree), 1, 0);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"climb rate PID",
		{"P",			"I",			"D"},
		{"pid climbrate p",	"pid climbrate i",	"pid climbrate d"},
		&cmdstree), 1, 1);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"altitude PID",
		{"P",			"I",			"D"},
		{"pid altitude p",	"pid altitude i",	"pid altitude d"},
		&cmdstree), 1, 2);

	tabs["filters"]->add_group(new float_settings_group(nullptr,
		"Complimentary filters",
		{"attitude",		"yaw",		"climb rate", 		"altitude"},
		{"compl attitude",	"compl yaw",	"compl climbrate",	"compl altitude"},
		&cmdstree), 3, 0, 2, 1); 
	tabs["filters"]->add_group(new float_settings_group(nullptr,
		"Low-pass filters",
		{"gyroscope",	"accelerometer",	"d-term",	"climb rate",	"acceleration",	"altitude"},
		{"lpf gyro",	"lpf accel",		"lpf d",	"lpf climb",	"lpf vaccel",	"lpf altitude"},
		&cmdstree), 3, 1, 2, 1);

	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"attitude offset",
		{"roll",	"pitch",	"yaw"},
		{"adj roll",	"adj pitch",	"adj yaw"},
		&cmdstree), 0, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"motor scale",
		{"roll",		"pitch"},
		{"adj rollthrust", 	"adj pitchthrust"},
		&cmdstree), 0, 1);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"accelerometer offset",
		{"X",		"Y",		"Z"},
		{"adj acc x",	"adj acc y",	"adj acc z"},
		&cmdstree), 1, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"gyroscope offset",
		{"X",		"Y",		"Z"},
		{"adj gyro x",	"adj gyro y",	"adj gyro z"},
		&cmdstree), 1, 1);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"magnetometer offsets",
		{"X",		"Y",		"Z",		"declination"},
		{"adj mag x0", 	"adj mag y0",	"adj mag z0", 	"adj mag decl"},
		&cmdstree), 2, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"magnetometer scale",
		{"X",			"Y",			"Z"},
		{"adj mag xscale", 	"adj mag yscale",	"adj mag zscale"},
		&cmdstree), 2, 1);

	tabs["control"]->add_group(new float_settings_group(nullptr,
		"control maximums",
		{"thrust",	"roll angle", 	"pitch angle",	"acceleration",	"altitude"},
		{"ctrl thrust",	"ctrl roll",	"ctrl pitch",	"ctrl accel",	"ctrl altmax"},
		&cmdstree), 3, 2, 2, 1);
	
	tabs["control"]->add_group(new float_settings_group(nullptr,
		"control rates",
		{"roll",	"pitch",	"yaw position",	"yaw",		"climb"},
		{"ctrl sroll",	"ctrl spitch",	"ctrl yaw",	"ctrl syaw",	"ctrl climbrate"},
		&cmdstree), 3, 3, 2, 1);

	settings_group *log_write = new settings_group(nullptr, "Log write");
	settings_group *log_read = new settings_group(nullptr, "Log read");

	log_write->add_setting(new uint_setting(nullptr,
		"records count"));
	log_write->add_setting_pair(
		new button_setting(nullptr, "start log"),
		new button_setting(nullptr, "stop log"));

	log_read->add_setting(new uint_setting(nullptr, "from"));
	log_read->add_setting(new uint_setting(nullptr, "to"));
	log_read->add_setting(new button_setting(nullptr, "read log"),
		false);
	log_read->add_setting(new button_setting(nullptr, "load log"),
		false);

	tabs["log"]->add_group(log_write, 0, 0, 1, 1);
	tabs["log"]->add_group(log_read, 0, 1, 1, 1);

	tab->addTab(tabs["pid"], "PID");
	tab->addTab(tabs["filters"], "Filters");
	tab->addTab(tabs["adjustments"], "Adjustments");
	tab->addTab(tabs["control"], "Control");
	tab->addTab(tabs["log"], "Log");


	settings["open"] = new button_setting(nullptr, "open config");
	settings["save"] = new button_setting(nullptr, "save config");
	settings["connect"] = new button_setting(nullptr, "connect to UAV");
	settings["flash"] = new button_setting(nullptr, "write to flash");
	
	connect(term->get_line(), &QLineEdit::returnPressed, this, &main_widget::return_pressed);
	connect(dynamic_cast<QPushButton *> (settings["open"]->get_field()),
		&QPushButton::clicked, this,
		&main_widget::open_click_handler);

	connect(dynamic_cast<QPushButton *> (settings["save"]->get_field()),
		&QPushButton::clicked, this,
		&main_widget::save_click_handler);

	connect(dynamic_cast<QPushButton *> (settings["connect"]->get_field()),
		&QPushButton::clicked, this,
		&main_widget::connect_click_handler);

	connect(dynamic_cast<QPushButton *> (settings["flash"]->get_field()),
		&QPushButton::clicked, this,
		&main_widget::flash_click_handler);

	connect(dynamic_cast<QPushButton *> (tabs["log"]
			->get_group("Log write")->get_setting("start log")->get_field()),
		&QPushButton::clicked, this,
		&main_widget::start_log_click_handler);

	connect(dynamic_cast<QPushButton *> (tabs["log"]
			->get_group("Log write")->get_setting("stop log")->get_field()),
		&QPushButton::clicked, this,
		&main_widget::stop_log_click_handler);

	connect(dynamic_cast<QPushButton *> (tabs["log"]
			->get_group("Log read")->get_setting("read log")->get_field()),
		&QPushButton::clicked, this,
		&main_widget::read_log_click_handler);

	connect(dynamic_cast<QPushButton *> (tabs["log"]
			->get_group("Log read")->get_setting("load log")->get_field()),
		&QPushButton::clicked, this,
		&main_widget::load_log_click_handler);


	QTimer *timer = new QTimer(this);
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
}
