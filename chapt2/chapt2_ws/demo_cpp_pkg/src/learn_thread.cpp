#include<iostream>
#include<thread>
#include<chrono>
#include<functional>
#include<cpp-httplib/httplib.h>

using namespace std;

class DownLoad{
    public:
    void download(const string &host, const string &path, const function<void(const string &, const string &)> &callback){
        cout<<"thread id = "<<this_thread::get_id()<<endl;
        httplib::Client client(host);
        auto response = client.Get(path);
        if(response && response->status == 200){
            callback(path, response->body);
        }
    }
    void start_download(const string &host, const string &path, const function<void(const string &, const string &)> &callback){
        auto download_func = bind(&DownLoad::download, this, placeholders::_1, placeholders::_2, placeholders::_3);
        thread download_thread(download_func, host, path, callback);
        download_thread.detach();
    }
};

int main(){
    DownLoad download;
    auto download_finish_callback = [](const string &path, const string &result){
        cout<<"finished download "<<path<<". get "<<result.length()<<" words. "<<result.substr(0,16)<<endl;;
    };
    download.start_download("http://localhost:8000", "/novel1.txt", download_finish_callback);
    download.start_download("http://localhost:8000", "/novel2.txt", download_finish_callback);
    download.start_download("http://localhost:8000", "/novel3.txt", download_finish_callback);
    this_thread::sleep_for(chrono::milliseconds(1000*10));
    return 0;
}