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

setting::setting(QWidget *parent, enum SETTING_TYPE t, string n,
	string c, commands_tree *cmdstree, vector<string> modes)
		: QWidget(parent)
{
	commands_tree *tr;
	stringstream ss;
	string tok;

	edit = NULL;
	box = NULL;
	button = NULL;
	label = new QLabel();

	name = n;
	command = c;

	if (cmdstree != NULL) {
		ss = stringstream(command);
		tr = cmdstree;

		while (getline(ss, tok, ' '))
			tr = tr->get_child(tok);
			
		tr->set_setting(this);
	}

	label->setFrameStyle(QFrame::Panel | QFrame::Sunken);
	label->setText(name.c_str());

	if (t == SETTING_TYPE_FLOAT) {
		edit = new QLineEdit();

		validator = new QRegExpValidator(	
			QRegExp("[-]{0,1}\\d{0,}\\.\\d{0,}"),0);

		edit->setValidator(validator);

		edit->setText(QString::fromStdString("0.0"));
	}
	else if (t == SETTING_TYPE_MODE) {
		box = new QComboBox();

		for (auto it = begin(modes); it != end(modes); ++it)
			box->addItem((*it).c_str());
	}
	else if (t == SETTING_TYPE_BUTTON)
		button = new QPushButton(QString::fromStdString(name));

	type = t;
}

setting::~setting()
{
	delete label;

	if (type == SETTING_TYPE_FLOAT) {
		delete edit;
		delete validator;
	}
	else if (type == SETTING_TYPE_MODE)	delete box;
	else if (type == SETTING_TYPE_BUTTON)	delete button;
}

string &setting::get_name()
{
	return name;
}

string &setting::get_command()
{
	return command;
}

QWidget *setting::get_label()
{
	return label;
}

QWidget *setting::get_field()
{
	switch (type) {
	case SETTING_TYPE_FLOAT:	return edit;
	case SETTING_TYPE_MODE:		return box;
	default:			return box;
	}
}

string setting::get_value() const
{
	if (type == SETTING_TYPE_FLOAT) {
		if (edit->text().toStdString().empty())
			return string("0.0");
		else
			return edit->text().toStdString();
	}
	else if (type == SETTING_TYPE_MODE) {
		return box->currentText().toStdString();
	}

	return string("");
}


void setting::set_value(const string &s)
{
	if (type == SETTING_TYPE_FLOAT) {
		string t;

		t = s;
		t.erase(t.find_last_not_of("0\r\n") + 1);

		if (t.back() == '.')
			t.pop_back();

		if (t.empty())
			t = "0";
		
		edit->setText(QString::fromStdString(t));
	}
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

void settings_group::add_setting(setting *s)
{
	settings[s->get_name()] = s;

	layout->addWidget(s->get_label(), layout_last, 0);
	layout->addWidget(s->get_field(), layout_last, 1);

	++layout_last;
}

string settings_group::get_setting_value(string name) const
{
	return settings.at(name)->get_value();
}

string settings_group::get_name()
{
	return name;
}

void settings_group::set_setting_value(string name, const string &v)
{
	settings.at(name)->set_value(v);
}


map<string, setting *> &settings_group::get_settings()
{
	return settings;
}

float_settings_group::float_settings_group(QWidget *parent,
	string name, vector<string> s, vector<string> c,
		commands_tree *cmdstree)
		: settings_group(parent, name)
{
	size_t i;

	for (i = 0; i < s.size(); ++i) {
		add_setting(new setting(nullptr,
			SETTING_TYPE_FLOAT, s[i], c[i], cmdstree));
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


map<string, settings_group *>  &settings_tab::get_groups()
{
	return groups;
}

settings_group *settings_tab::get_group(string name)
{
	return groups[name];
}

const setting *settings_tab::get_setting(string name)
{
	return settings[name];
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

}

void terminal::add_output(string s)
{
	edit->append(QString::fromStdString(s));
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

		if (tr->get_setting() != NULL)
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
	
	term->add_output(string("loading values..."));

	conf = string("");

	for (auto t = begin(tabs); t != end(tabs); ++t) {
		map<string, settings_group *> &groups
			= t->second->get_groups();

		for (auto g = begin(groups); g != end(groups); ++g) {
			map<string, setting *> &settings
				= g->second->get_settings();

			for (auto s = begin(settings); s != end(settings); ++s) {
				sendcmd(lsfd, &rsi,
					("get " + s->second->get_command()).c_str(),
					getfunc, write_conf, (void *) &conf);
			
					
				term->add_output("get " + s->second->get_command());
				
				QCoreApplication::processEvents();
			}
		}
	}

	term->add_output(string("Connected to UAV, values loaded"));

	string_to_conf(conf);
}

void write_term(void *t, const char *s)
{
	terminal *term;

	term = (terminal *) t;

	term->add_output(string(s));
	QCoreApplication::processEvents();
}

void main_widget::flash_click_handler()
{
	stringstream ss;
	string line;
	
	conf_to_string();

	ss = stringstream(conf);

	while (getline(ss, line, '\n')) {
		sendcmd(lsfd, &rsi, line.c_str(), conffunc,
			write_term, (void *) term);
	
		QCoreApplication::processEvents();
	}
}

commands_tree::commands_tree(const string &n) : cmdsetting(NULL)
{
	name = n;
}

commands_tree::~commands_tree()
{

}

commands_tree *commands_tree::get_child(const string &s)
{
	if (children.count(s) == 0)
		children[s] = new commands_tree(s);
	
	return children[s];
}

setting *commands_tree::get_setting()
{
	return cmdsetting;
}

void commands_tree::set_setting(setting *s)
{
	cmdsetting = s;
}

map<string, commands_tree *> &commands_tree::get_children()
{
	return children;
}

main_widget::main_widget(QWidget *parent) : QWidget(parent)
{
	grid = new QGridLayout(this);
	tab = new QTabWidget;

	tabs["pid"] = new settings_tab;
	tabs["adjustments"] = new settings_tab;
	tabs["filters"] = new settings_tab;
	tabs["control"] = new settings_tab;

	initsocks(&lsfd, &rsi);

	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"rate PID", {"P", "I", "D"},
		{"pid stilt p", "pid stilt i", "pid stilt d"},
		&cmdstree), 0, 0);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"angle PID", {"P", "I", "D"},
		{"pid tilt p", "pid tilt i", "pid tilt d"}, &cmdstree),
		0, 1);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"yaw PID", {"P", "I", "D"},
		{"pid syaw p", "pid syaw i", "pid syaw d"}, &cmdstree),
		0, 2);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"yaw position PID", {"P", "I", "D"},
		{"pid yaw p", "pid yaw i", "pid yaw d"}, &cmdstree),
		0, 3);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"throttle PID", {"P", "I", "D"},
		{"pid throttle p", "pid throttle i", "pid throttle d"},
		&cmdstree), 1, 0);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"climb rate PID", {"P", "I", "D"},
		{"pid climbrate p", "pid climbrate i",
		"pid climbrate d"}, &cmdstree), 1, 1);
	tabs["pid"]->add_group(new float_settings_group(nullptr,
		"altitude PID", {"P", "I", "D"},
		{"pid altitude p", "pid altitude i", "pid altitude d"},
		&cmdstree), 1, 2);

	tabs["filters"]->add_group(new float_settings_group(nullptr,
		"Complimentary filters",
		{"attitude", "yaw", "climb rate", "altitude"},
		{"compl attitude", "compl yaw",
		"compl climbrate", "compl altitude"}, &cmdstree),
		3, 0, 2, 1); 
	tabs["filters"]->add_group(new float_settings_group(nullptr,
		"Low-pass filters",
		{"gyroscope", "accelerometer", "d-term",
		"climb rate", "acceleration", "altitude"},
		{"lpf gyro", "lpf accel", "lpf d", "lpf climb",
		"lpf vaccel", "lpf altitude"}, &cmdstree), 3, 1, 2, 1);

	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"attitude offset", {"roll", "pitch", "yaw"},
		{"adj roll", "adj pitch", "adj yaw"}, &cmdstree), 0, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"motor scale", {"roll", "pitch"},
		{"adj rollthrust", "adj pitchthrust"}, &cmdstree),
		0, 1);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"accelerometer offset", {"X", "Y", "Z"},
		{"adj acc x", "adj acc y", "adj acc z"}, &cmdstree),
		1, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"gyroscope offset", {"X", "Y", "Z"},
		{"adj gyro x", "adj gyro y", "adj gyro z"}, &cmdstree),
		1, 1);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"magnetometer offsets", {"X", "Y", "Z", "declination"},
		{"adj mag x0", "adj mag y0",
		"adj mag z0", "adj mag decl"}, &cmdstree), 2, 0);
	tabs["adjustments"]->add_group(new float_settings_group(nullptr,
		"magnetometer scale", {"X", "Y", "Z"},
		{"adj mag xscale", "adj mag yscale",
		"adj mag zscale"}, &cmdstree), 2, 1);

	tabs["control"]->add_group(new float_settings_group(nullptr,
		"control maximums",
		{"thrust", "roll angle", "pitch angle",
		"acceleration", "altitude"},
		{"ctrl thrust", "ctrl roll", "ctrl pitch",
		"ctrl accel", "ctrl altmax"}, &cmdstree), 3, 2, 2, 1);
	
	tabs["control"]->add_group(new float_settings_group(nullptr,
		"control rates",
		{"roll", "pitch", "yaw position", "yaw", "climb"},
		{"ctrl sroll", "ctrl spitch", "ctrl yaw",
		"ctrl syaw", "ctrl climbrate"}, &cmdstree), 3, 3, 2, 1);

	tab->addTab(tabs["pid"], "PID");
	tab->addTab(tabs["filters"], "Filters");
	tab->addTab(tabs["adjustments"], "Adjustments");
	tab->addTab(tabs["control"], "Control");

	settings["open"] = new setting(nullptr, SETTING_TYPE_BUTTON,
		"open config");
	settings["save"] = new setting(nullptr, SETTING_TYPE_BUTTON,
		"save config");
	settings["connect"] = new setting(nullptr, SETTING_TYPE_BUTTON,
		"connect to UAV");
	settings["flash"] = new setting(nullptr, SETTING_TYPE_BUTTON,
		"write to flash");

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

	term = new terminal();

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
