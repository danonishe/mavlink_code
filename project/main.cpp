
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
if (!fl)
{
    cout<<"unknown command"<<endl;
}
 }

t.join(); 
 
    return 0;
}