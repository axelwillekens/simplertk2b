#include <iostream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <json/json.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <thread>

using namespace std;

//global var
int clientSocket;
int listening;
ros::NodeHandle n;


void gpsGateway(const sensor_msgs::NavSatFixConstPtr& msg);
void doGPSWork(void);
void doPIDWork(void);

int main(int argc, char **argv) {


    //init ros
    ros::init(argc,argv,"gateway");


    //STAGE SOCKET CONNECTION
    listening = socket(AF_INET,SOCK_STREAM,0);
    if(listening == -1)
    {
        cerr << "can't create a socket";
        return -1;
        //exit program?
    }
    sockaddr_in hint;
    hint.sin_family = AF_INET;
    hint.sin_port = htons(5120);
    inet_pton(AF_INET,"0.0.0.0", &hint.sin_addr);

    if(bind(listening,(sockaddr*)&hint, sizeof(hint))==-1){
        cerr << "can't bind to IP/port";
        return -2;
    }
    //LISTEN
    if(listen(listening,SOMAXCONN)==-1){
        cerr << "Can't listen";
        return -4;
    }
    //ACCEPT CALLS
    sockaddr_in client;
    socklen_t clientSize;
    char host[NI_MAXHOST];
    char svc[NI_MAXSERV];

    cout << "waiting for connection..." << endl;
    clientSocket = accept(listening,(sockaddr*)&client,&clientSize);
    cout << "connection accepted" <<endl;
    if(clientSocket ==-1){
        cerr << "Problem with client connecting!";
        return -4;
    }
    else{
        cout << "connection established" << endl;
    }
    //CLOSE
    close(listening);
    memset(host,0,NI_MAXHOST);
    memset(svc,0,NI_MAXSERV);
    int result = getnameinfo((sockaddr*)&client,
                                    sizeof(client),
                                    host,
                                    NI_MAXHOST,
                                    svc,
                                    NI_MAXSERV,
                                    0);
    if(result)
    {
        cout << host << " connected on " << svc << endl;
    }
    else
    {
        inet_ntop(AF_INET , &client.sin_addr,host,NI_MAXHOST);
        cout << host << " connected on " << ntohs(client.sin_port) <<endl;
    }

    //launch two threads? One PID | One GPS 
    std::thread tGPS(doGPSWork);
    std::thread tPID(doPIDWork);    
    } 


void doGPSWork(){
    //initiate Subscriber and callback gpsGateway
    ros::Subscriber sub = n.subscribe("gga_pub",1000,gpsGateway);
    ros::spin();
}
void gpsGateway(const sensor_msgs::NavSatFixConstPtr& msg){

    //testing receival
    ROS_INFO("received from gps: [%s]", msg->data.c_str);
    //extract lat and lon from msg-->added by gga_pub so hope I can get it off also
    double lat = msg.latitude;
    double lon = msg.longitude;
    std::map<string, double> coords = {"lat":lat,"lon":lon};
    //convert to json and send over socket
    string mapstring = coords.asString();
    send(listening, mapstring,strlen(mapstring),0);
    }


void doPIDWork(){
    //initiate publisher 
    //TODO check if correct UInt32 type used--> maybe a pointer or container allocator is needed?
    
    ros::Publisher pidpub = n.advertise<std_msgs::UInt32>("pid_pub",1000);

    //databuffer (chuncks-size)
    char buf[4096];
    while (true){

        //clear the buffer 
        memset(buf,0,4096);

        //Receive PID data over socket
        int bytesRecv = recv(clientSocket, buf,4096,0);
        if(bytesRecv == -1){
            cerr << "There was a connection issue" << endl;
            break;
        }
        if(bytesRecv == 0)
        {
            cout << "The client disconnected";
        }

        //test 123 ...OK!
        cout << "PID values: " << string(buf,0,bytesRecv) << endl;

        //store in string (is this a valid jsonstring?)
        string jsonstring = string(buf,0,bytesRecv);

        //parse incoming jsonstring
        Json::Value root;
        Json::Reader reader;
        bool parsingSuccesful = reader.parse(jsonstring,root);
        if(!parsingSuccesful){
            cout << "failed to parse pid-values jsonstring\n"
            << reader.getFormattedErrorMessages() << endl;
        }

        //extract and convert
        int P = stoi(root.get("P", "UTF-8").asString());
        int I = stoi(root.get("I", "UTF-8").asString());
        int D = stoi(root.get("D", "UTF-8").asString());
        //store into UINT32 array
        std_msgs::UInt32 piddata= {P,I,D};
        //publish
        pidpub.publish(piddata);
        //no idea if this is mandatory...?
        ros.spinOnce();
    }
}
