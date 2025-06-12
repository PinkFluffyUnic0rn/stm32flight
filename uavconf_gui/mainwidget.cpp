#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#include <QtWidgets>
#include "mainwidget.h"

using namespace std;

setting::setting(QWidget *parent, enum SETTING_TYPE t, string n,
	vector<string> modes) : QWidget(parent)
{
	name = n;
	
	edit = NULL;
	box = NULL;
	button = NULL;
	label = new QLabel();

	label->setFrameStyle(QFrame::Panel | QFrame::Sunken);
	label->setText(name.c_str());

	if (t == SETTING_TYPE_FLOAT) {
		edit = new QLineEdit();

		validator = new QRegExpValidator(	
			QRegExp("[-]{0,1}\\d{0,}\\.\\d{0,}"),0);

		edit->setValidator(validator);
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
	if (type == SETTING_TYPE_FLOAT)
		edit->setText(QString::fromStdString(s));

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


float_settings_group::float_settings_group(QWidget *parent,
	string name, vector<string> s) : settings_group(parent, name)
{
	for (auto it = begin(s); it != end(s); ++it)
		add_setting(new setting(nullptr, SETTING_TYPE_FLOAT, *it));
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
	const settings_group *g;

	conf = string("");

	g = pid_tab->get_group("angle PID");
	conf += string("pid tilt p ") + g->get_setting_value("P") + string("\n");
	conf += string("pid tilt i ") + g->get_setting_value("I") + string("\n");
	conf += string("pid tilt d ") + g->get_setting_value("D") + string("\n");

	g = pid_tab->get_group("rate PID");
	conf += string("pid stilt p ") + g->get_setting_value("P") + string("\n");
	conf += string("pid stilt i ") + g->get_setting_value("I") + string("\n");
	conf += string("pid stilt d ") + g->get_setting_value("D") + string("\n");

	g = pid_tab->get_group("yaw PID");
	conf += string("pid syaw p ") + g->get_setting_value("P") + string("\n");
	conf += string("pid syaw i ") + g->get_setting_value("I") + string("\n");
	conf += string("pid syaw d ") + g->get_setting_value("D") + string("\n");

	g = pid_tab->get_group("yaw position PID");
	conf += string("pid yaw p ") + g->get_setting_value("P") + string("\n");
	conf += string("pid yaw i ") + g->get_setting_value("I") + string("\n");
	conf += string("pid yaw d ") + g->get_setting_value("D") + string("\n");

	g = pid_tab->get_group("throttle PID");
	conf += string("pid throttle p ") + g->get_setting_value("P") + string("\n");
	conf += string("pid throttle i ") + g->get_setting_value("I") + string("\n");
	conf += string("pid throttle d ") + g->get_setting_value("D") + string("\n");

	g = pid_tab->get_group("climb rate PID");
	conf += string("pid climbrate p ") + g->get_setting_value("P") + string("\n");
	conf += string("pid climbrate i ") + g->get_setting_value("I") + string("\n");
	conf += string("pid climbrate d ") + g->get_setting_value("D") + string("\n");

	g = pid_tab->get_group("altitude PID");
	conf += string("pid altitude p ") + g->get_setting_value("P") + string("\n");
	conf += string("pid altitude i ") + g->get_setting_value("I") + string("\n");
	conf += string("pid altitude d ") + g->get_setting_value("D") + string("\n");


	g = filters_tab->get_group("Complimentary filters");
	conf += string("compl attitude ") + g->get_setting_value("attitude") + string("\n");
	conf += string("compl yaw ") + g->get_setting_value("yaw") + string("\n");
	conf += string("compl climbrate ") + g->get_setting_value("climb rate") + string("\n");
	conf += string("compl altitude ") + g->get_setting_value("altitude") + string("\n");

	g = filters_tab->get_group("Low-pass filters");
	conf += string("lpf gyro ") + g->get_setting_value("gyroscope") + string("\n");
	conf += string("lpf accel ") + g->get_setting_value("accelerometer") + string("\n");
	conf += string("lpf d ") + g->get_setting_value("d-term") + string("\n");
	conf += string("lpf climb ") + g->get_setting_value("climb rate") + string("\n");
	conf += string("lpf vaccel ") + g->get_setting_value("acceleration") + string("\n");
	conf += string("lpf altitude ") + g->get_setting_value("altitude") + string("\n");


	g = adjustments_tab->get_group("motor scale");
	conf += string("adj rollthrust ") + g->get_setting_value("roll") + string("\n");
	conf += string("adj pitchthrust ") + g->get_setting_value("pitch") + string("\n");

	g = adjustments_tab->get_group("attitude offset");
	conf += string("adj roll ") + g->get_setting_value("roll") + string("\n");
	conf += string("adj pitch ") + g->get_setting_value("pitch") + string("\n");
	conf += string("adj yaw ") + g->get_setting_value("yaw") + string("\n");

	g = adjustments_tab->get_group("accelerometer offset");
	conf += string("adj acc x ") + g->get_setting_value("X") + string("\n");
	conf += string("adj acc y ") + g->get_setting_value("Y") + string("\n");
	conf += string("adj acc z ") + g->get_setting_value("Z") + string("\n");

	g = adjustments_tab->get_group("gyroscope offset");
	conf += string("adj gyro x ") + g->get_setting_value("X") + string("\n");
	conf += string("adj gyro y ") + g->get_setting_value("Y") + string("\n");
	conf += string("adj gyro z ") + g->get_setting_value("Z") + string("\n");

	g = adjustments_tab->get_group("magnetometer offsets");
	conf += string("adj mag x0 ") + g->get_setting_value("X") + string("\n");
	conf += string("adj mag y0 ") + g->get_setting_value("Y") + string("\n");
	conf += string("adj mag z0 ") + g->get_setting_value("Z") + string("\n");
	conf += string("adj mag decl ") + g->get_setting_value("declination") + string("\n");

	g = adjustments_tab->get_group("magnetometer scale");
	conf += string("adj mag xscale ") + g->get_setting_value("X") + string("\n");
	conf += string("adj mag yscale ") + g->get_setting_value("Y") + string("\n");
	conf += string("adj mag zscale ") + g->get_setting_value("Z") + string("\n");


	g = control_tab->get_group("control maximums");
	conf += string("ctrl thrust ") + g->get_setting_value("thrust") + string("\n");
	conf += string("ctrl roll ") + g->get_setting_value("roll angle") + string("\n");
	conf += string("ctrl pitch ") + g->get_setting_value("pitch angle") + string("\n");
	conf += string("ctrl accel ") + g->get_setting_value("acceleration") + string("\n");
	conf += string("ctrl altmax ") + g->get_setting_value("altitude") + string("\n");

	g = control_tab->get_group("control rates");
	conf += string("ctrl sroll ") + g->get_setting_value("roll") + string("\n");
	conf += string("ctrl spitch ") + g->get_setting_value("pitch") + string("\n");
	conf += string("ctrl syaw ") + g->get_setting_value("yaw") + string("\n");
	conf += string("ctrl yaw ") + g->get_setting_value("yaw position") + string("\n");
	conf += string("ctrl climbrate ") + g->get_setting_value("climb") + string("\n");

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

		while (getline(ss, tok, ' ')) {
			toks.push_back(tok);
		}
		

		if (toks[0] == "pid") {
			if (toks[1] == "tilt") {
				settings_group *g;

				g = pid_tab->get_group("angle PID");

				if (toks[2] == "p")
					g->set_setting_value("P", toks[3]);
				else if (toks[2] == "i")
					g->set_setting_value("I", toks[3]);
				else if (toks[2] == "d")
					g->set_setting_value("D", toks[3]);
			}
			else if (toks[1] == "stilt") {
				settings_group *g;

				g = pid_tab->get_group("rate PID");

				if (toks[2] == "p")
					g->set_setting_value("P", toks[3]);
				else if (toks[2] == "i")
					g->set_setting_value("I", toks[3]);
				else if (toks[2] == "d")
					g->set_setting_value("D", toks[3]);
			}
			else if (toks[1] == "yaw") {
				settings_group *g;

				g = pid_tab->get_group("yaw position PID");

				if (toks[2] == "p")
					g->set_setting_value("P", toks[3]);
				else if (toks[2] == "i")
					g->set_setting_value("I", toks[3]);
				else if (toks[2] == "d")
					g->set_setting_value("D", toks[3]);
			}
			else if (toks[1] == "syaw") {
				settings_group *g;

				g = pid_tab->get_group("yaw PID");

				if (toks[2] == "p")
					g->set_setting_value("P", toks[3]);
				else if (toks[2] == "i")
					g->set_setting_value("I", toks[3]);
				else if (toks[2] == "d")
					g->set_setting_value("D", toks[3]);
			}
			else if (toks[1] == "throttle") {
				settings_group *g;

				g = pid_tab->get_group("throttle PID");

				if (toks[2] == "p")
					g->set_setting_value("P", toks[3]);
				else if (toks[2] == "i")
					g->set_setting_value("I", toks[3]);
				else if (toks[2] == "d")
					g->set_setting_value("D", toks[3]);
			}
			else if (toks[1] == "climbrate") {
				settings_group *g;

				g = pid_tab->get_group("climb rate PID");

				if (toks[2] == "p")
					g->set_setting_value("P", toks[3]);
				else if (toks[2] == "i")
					g->set_setting_value("I", toks[3]);
				else if (toks[2] == "d")
					g->set_setting_value("D", toks[3]);
			}
			else if (toks[1] == "altitude") {
				settings_group *g;

				g = pid_tab->get_group("altitude PID");

				if (toks[2] == "p")
					g->set_setting_value("P", toks[3]);
				else if (toks[2] == "i")
					g->set_setting_value("I", toks[3]);
				else if (toks[2] == "d")
					g->set_setting_value("D", toks[3]);
			}
		}
		else if (toks[0] == "compl") {
			settings_group *g;
			
			g = filters_tab->get_group("Complimentary filters");
			
			if (toks[1] == "attitude")
				g->set_setting_value("attitude", toks[2]);
			else if (toks[1] == "yaw")
				g->set_setting_value("yaw", toks[2]);
			else if (toks[1] == "climbrate")
				g->set_setting_value("climb rate", toks[2]);
			else if (toks[1] == "altitude")
				g->set_setting_value("altitude", toks[2]);
		}
		else if (toks[0] == "lpf") {
			settings_group *g;
			
			g = filters_tab->get_group("Low-pass filters");
			
			if (toks[1] == "gyro")
				g->set_setting_value("gyroscope", toks[2]);
			else if (toks[1] == "accel")
				g->set_setting_value("accelerometer", toks[2]);
			else if (toks[1] == "d")
				g->set_setting_value("d-term", toks[2]);
			else if (toks[1] == "climb")
				g->set_setting_value("climb rate", toks[2]);
			else if (toks[1] == "vaccel")
				g->set_setting_value("acceleration", toks[2]);
			else if (toks[1] == "altitude")
				g->set_setting_value("altitude", toks[2]);
		}
		else if (toks[0] == "adj") {
			settings_group *g;
			
			if (toks[1] == "rollthrust") {
				g = adjustments_tab->get_group("motor scale");
				g->set_setting_value("roll", toks[3]);
			}
			else if (toks[1] == "pitchthrust") {
				g = adjustments_tab->get_group("motor scale");
				g->set_setting_value("pitch", toks[3]);
			}
			else if (toks[1] == "roll") {
				g = adjustments_tab->get_group("attitude offset");
				g->set_setting_value("roll", toks[3]);
			}
			else if (toks[1] == "pitch") {
				g = adjustments_tab->get_group("attitude offset");
				g->set_setting_value("pitch", toks[3]);
			}
			else if (toks[1] == "yaw") {
				g = adjustments_tab->get_group("attitude offset");
				g->set_setting_value("yaw", toks[3]);
			}
			else if (toks[1] == "acc") {
				g = adjustments_tab->get_group("accelerometer offset");
				if (toks[2] == "x")
					g->set_setting_value("X", toks[3]);
				if (toks[2] == "y")
					g->set_setting_value("Y", toks[3]);
				if (toks[2] == "z")
					g->set_setting_value("Z", toks[3]);
			}
			else if (toks[1] == "gyro") {
				g = adjustments_tab->get_group("gyroscope offset");
				if (toks[2] == "x")
					g->set_setting_value("X", toks[3]);
				if (toks[2] == "y")
					g->set_setting_value("Y", toks[3]);
				if (toks[2] == "z")
					g->set_setting_value("Z", toks[3]);
			}
			else if (toks[1] == "mag") {
				if (toks[2] == "x0") {
					g = adjustments_tab->get_group("magnetometer offsets");
					g->set_setting_value("X", toks[3]);
				}
				else if (toks[2] == "y0") {
					g = adjustments_tab->get_group("magnetometer offsets");
					g->set_setting_value("Y", toks[3]);
				}
				else if (toks[2] == "z0") {
					g = adjustments_tab->get_group("magnetometer offsets");
					g->set_setting_value("Z", toks[3]);
				}
				else if (toks[2] == "decl") {
					g = adjustments_tab->get_group("magnetometer offsets");
					g->set_setting_value("declination", toks[3]);
				}
				else if (toks[2] == "xscale") {
					g = adjustments_tab->get_group("magnetometer scale");
					g->set_setting_value("X", toks[3]);
				}
				else if (toks[2] == "yscale") {
					g = adjustments_tab->get_group("magnetometer scale");
					g->set_setting_value("Y", toks[3]);
				}
				else if (toks[2] == "zscale") {
					g = adjustments_tab->get_group("magnetometer scale");
					g->set_setting_value("Z", toks[3]);
				}
			}

		}
		else if (toks[0] == "ctrl") {
			settings_group *g;

			if (toks[1] == "thrust") {
				g = control_tab->get_group("control maximums");
				g->set_setting_value("thrust", toks[2]);
			}
			else if (toks[1] == "roll") {
				g = control_tab->get_group("control maximums");
				g->set_setting_value("roll angle", toks[2]);
			}
			else if (toks[1] == "pitch") {
				g = control_tab->get_group("control maximums");
				g->set_setting_value("pitch angle", toks[2]);
			}
			else if (toks[1] == "accel") {
				g = control_tab->get_group("control maximums");
				g->set_setting_value("acceleration", toks[2]);
			}
			else if (toks[1] == "altmax") {
				g = control_tab->get_group("control maximums");
				g->set_setting_value("altitude", toks[2]);
			}
			else if (toks[1] == "sroll") {
				g = control_tab->get_group("control rates");
				g->set_setting_value("roll", toks[2]);
			}
			else if (toks[1] == "spitch") {
				g = control_tab->get_group("control rates");
				g->set_setting_value("pitch", toks[2]);
			}
			else if (toks[1] == "syaw") {
				g = control_tab->get_group("control rates");
				g->set_setting_value("yaw", toks[2]);
			}
			else if (toks[1] == "yaw") {
				g = control_tab->get_group("control rates");
				g->set_setting_value("yaw position", toks[2]);
			}
			else if (toks[1] == "climbrate") {
				g = control_tab->get_group("control rates");
				g->set_setting_value("climb", toks[2]);
			}
		}
			
	}
}

void main_widget::flash_click_handler()
{
	conf_to_string();

	term->add_output(conf);
}

void main_widget::open_click_handler()
{
	QString name;
	ifstream infile;
	stringstream buf;

	name = QFileDialog::getOpenFileName(this, tr("Open file"), "", tr("Text files (*.txt)"));

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

main_widget::main_widget(QWidget *parent) : QWidget(parent)
{
	grid = new QGridLayout(this);
	tab = new QTabWidget;

	pid_tab = new settings_tab;
	adjustments_tab = new settings_tab;
	filters_tab = new settings_tab;
	control_tab = new settings_tab;

	pid_tab->add_group(new float_settings_group(nullptr,
		"rate PID", {"P", "I", "D"}), 0, 0);
	pid_tab->add_group(new float_settings_group(nullptr,
		"angle PID", {"P", "I", "D"}), 0, 1);
	pid_tab->add_group(new float_settings_group(nullptr,
		"yaw PID", {"P", "I", "D"}), 0, 2);
	pid_tab->add_group(new float_settings_group(nullptr,
		"yaw position PID", {"P", "I", "D"}), 0, 3);
	pid_tab->add_group(new float_settings_group(nullptr,
		"throttle PID", {"P", "I", "D"}), 1, 0);
	pid_tab->add_group(new float_settings_group(nullptr,
		"climb rate PID", {"P", "I", "D"}), 1, 1);
	pid_tab->add_group(new float_settings_group(nullptr,
		"altitude PID", {"P", "I", "D"}), 1, 2);

	filters_tab->add_group(new float_settings_group(nullptr,
		"Complimentary filters",
		{"attitude", "yaw", "climb rate", "altitude"}),
		3, 0, 2, 1); 

	filters_tab->add_group(new float_settings_group(nullptr,
		"Low-pass filters",
		{"gyroscope", "accelerometer", "d-term",
		"climb rate", "acceleration", "altitude"}), 3, 1, 2, 1);

	adjustments_tab->add_group(new float_settings_group(nullptr,
		"attitude offset", {"roll", "pitch", "yaw"}), 0, 0);
	adjustments_tab->add_group(new float_settings_group(nullptr,
		"motor scale", {"roll", "pitch"}), 0, 1);
	adjustments_tab->add_group(new float_settings_group(nullptr,
		"accelerometer offset", {"X", "Y", "Z"}), 1, 0);
	adjustments_tab->add_group(new float_settings_group(nullptr,
		"gyroscope offset", {"X", "Y", "Z"}), 1, 1);
	adjustments_tab->add_group(new float_settings_group(nullptr,
		"magnetometer offsets", {"X", "Y", "Z", "declination"}),
		2, 0);
	adjustments_tab->add_group(new float_settings_group(nullptr,
		"magnetometer scale", {"X", "Y", "Z"}), 2, 1);

	control_tab->add_group(new float_settings_group(nullptr,
		"control maximums",
		{"thrust", "roll angle", "pitch angle",
		"acceleration", "altitude"}), 3, 2, 2, 1);
	
	control_tab->add_group(new float_settings_group(nullptr,
		"control rates",
		{"roll", "pitch", "yaw position", "yaw", "climb"}), 3, 3, 2, 1);

	tab->addTab(pid_tab, "PID");
	tab->addTab(filters_tab, "Filters");
	tab->addTab(adjustments_tab, "Adjustments");
	tab->addTab(control_tab, "Control");

	open = new setting(nullptr, SETTING_TYPE_BUTTON, "open config");
	save = new setting(nullptr, SETTING_TYPE_BUTTON, "save config");

	flash = new setting(nullptr, SETTING_TYPE_BUTTON,
		"write to flash");

	connect(dynamic_cast<QPushButton *> (open->get_field()),
		&QPushButton::clicked, this,
		&main_widget::open_click_handler);

	connect(dynamic_cast<QPushButton *> (save->get_field()),
		&QPushButton::clicked, this,
		&main_widget::save_click_handler);

	connect(dynamic_cast<QPushButton *> (flash->get_field()),
		&QPushButton::clicked, this,
		&main_widget::flash_click_handler);

/*
	startlog = new setting(nullptr, SETTING_TYPE_BUTTON, "start log");
	stoplog = new setting(nullptr, SETTING_TYPE_BUTTON, "stop log");
	loadlog = new setting(nullptr, SETTING_TYPE_BUTTON, "load log");
*/
	term = new terminal();

	grid->addWidget(tab, 0, 0, 5, 4);
	grid->addWidget(open->get_field(), 5, 0);
	grid->addWidget(save->get_field(), 5, 1);
	grid->addWidget(flash->get_field(), 5, 3);
	grid->addWidget(term, 0, 4, 6, 2);
	
	/*
	grid->addWidget(startlog->get_field(), 5, 1);
	grid->addWidget(stoplog->get_field(), 5, 2);
	grid->addWidget(loadlog->get_field(), 5, 3);
	*/

	setWindowTitle(tr("Settings"));
}

main_widget::~main_widget()
{
}
