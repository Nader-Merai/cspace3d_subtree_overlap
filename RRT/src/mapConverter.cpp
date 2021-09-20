#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>


int main (int argc, char* argv[]) {
    std::ifstream input;
    std::ofstream output;
    std::string buf;
    
    char* ipt = argv[1];
    char* opt = argv[2];

    input.open(ipt,std::ios::in);
    output.open(opt,std::ios::out);

    if(!input.is_open()) {
        std::string err(argv[0]);
        std::cout << err << " file open failed.\n";
        return 0;
    }
    // omit the first line
    getline(input, buf);
    
    // get configuration
    int h(0), w(0);
    getline(input, buf);
    h = atoi(buf.substr(7).c_str());

    getline(input, buf);
    w = atoi(buf.substr(6).c_str());

    // omit the 4th line
    getline(input, buf);
    output << "discretization(cells): " << w << " " << h << std::endl;
    output << "obsthresh: 1" << std::endl;
    output << "cost_inscribed_thresh: 1" << std::endl;
    output << "cost_possibly_circumscribed_thresh: 0" <<  std::endl;
    output << "cellsize(meters): 0.025" << std::endl;
    output << "nominalvel(mpersecs): 1.0" << std::endl;
    output << "timetoturn45degsinplace(secs): 2.0" << std::endl;
    output << "start(meters,rads): 0.11 0.11 0" << std::endl;
    output << "end(meters,rads): " << (double)(w-40) * 0.025 << " "
        << (double)(h-40) * 0.025 << " 0"<< std::endl;
    output << "environment:" << std::endl;

    while(getline(input, buf)) {
        std::replace(buf.begin(), buf.end(), '.', '0');
        std::replace(buf.begin(), buf.end(), 'G', '0');
        std::replace(buf.begin(), buf.end(), '@', '1');
        std::replace(buf.begin(), buf.end(), 'O', '1');
        std::replace(buf.begin(), buf.end(), 'T', '1');
        std::replace(buf.begin(), buf.end(), 'S', '0');
        std::replace(buf.begin(), buf.end(), 'W', '1');

        for( int i = 1; i < buf.size(); ++i ){
            buf.insert(i, " ");
            ++i;
        }
        output << buf;
        output << std::endl;
    } 

    input.close();
    output.close();
    return 0;
}
