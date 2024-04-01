
#include<iostream>
#include " class.h"

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


while (true){

cout<<"Commands: \n 1 - takeoff \n 2 - land\n 3 - return to lunch\nEnter num: ";
int a;
cin>>a;
cout<<endl;
switch(a){
    case 1:
    {
        if(!mav.takeoff(10))
        {
            cout<<"success takeoff\n";
        }
        else
        {
             cout<<"error takeoff\n";
        }
        break;
    }
    case 2:
    {
         if(!mav.land())
        {
            cout<<"success land\n";
        }
        else
        {
             cout<<"error land\n";
        }
        break;
    }
     case 3:
    {
         if(!mav.return_to_lunch())
        {
            cout<<"success return to lunch\n";
        }
        else
        {
             cout<<"error return to lunch\n";
        }
        break;
    }
     default: 
        cout << "unknown command\n";
        break;
    }
    cout<<endl;
}

t.join(); 
 
    return 0;
}