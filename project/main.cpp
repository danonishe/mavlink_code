
#include<iostream>
#include " class.h"
#include<string>
#include<vector>


int main(int argc, char* argv[])
{
  MavlinkReceiver mav;
  if(mav.connect()!=0)
  {
    cout<<"error"<<endl;
    return 0;
  }

thread t([&](){mav.ProcessData();});

this_thread::sleep_for(chrono::milliseconds(100));

vector<string> commander;
while (true){

cout<<"Command: ";
string str;
getline(cin, str);

commander.clear();

 size_t pos = 0;
  while ((pos = str.find(' ')) != string::npos) {
    commander.push_back(str.substr(0, pos));
    str.erase(0, pos+1);
  }
  if (!str.empty()&& str.find(' ')== string::npos)
  {
    commander.push_back(str);
  }

bool fl = 0;

if (commander[0]=="/takeoff")
{
    if (commander.size()>1)
    {
        (!mav.takeoff(stof(commander[1])))? cout<<"success takeoff\n": cout<<"error takeoff\n";
    }else
    {
        cout<<"Высота по умолчанию 10м"<<endl;
        (!mav.takeoff(10.0))? cout<<"success takeoff\n": cout<<"error takeoff\n";
    }
    fl=1;
}

if (commander[0]=="/speed")
{
    if (commander.size()>1)
    {
        (!mav.change_speed(stof(commander[1])))? cout<<"success change speed\n": cout<<"error change speed\n";
    }else
    {
        cout<<"скорость по умолчанию 5м/с"<<endl;
        (!mav.change_speed(5.0))? cout<<"success change speed\n": cout<<"error change speed\n";
    }
    fl=1;
}


if (commander[0]=="/land")
{

 (!mav.land())?cout<<"success land\n":cout<<"error land\n";
 fl=1;
}

if (commander[0]=="/rtl")
{
    (!mav.return_to_lunch())?cout<<"success return to lunch\n":cout<<"error return to lunch\n";
    fl=1;
}

if (commander[0]=="/load")
{
  

 if (commander.size()>1)
    {
        (!mav.parse_mission_json(commander[1]))? cout<<"success load mission\n": cout<<"error load mission\n";
    }else
    {
        cout<<"название файла не написано"<<endl;
    }
 fl=1;
}

if (commander[0]=="/upload")
{
    (!mav.upload_mission())?cout<<"success upload_mission\n":cout<<"error upload_mission\n";
    fl=1;
}
if (commander[0]=="/download")
{
    (!mav.download_mission())?cout<<"success download_mission\n":cout<<"error download_mission\n";
    fl=1;
}

if (commander[0]=="/set")
{
    (!mav.set_mission())?cout<<"success set_mission\n":cout<<"error set_mission\n";
    fl=1;
}

if (!fl)
{
    cout<<"unknown command"<<endl;
}
 }

t.join(); 
 
    return 0;
}