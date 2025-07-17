#include <iostream>
#include <functional>

using namespace std;

void free_func(const string &str)
{
    cout << str << endl;
}

class PrintStr
{
public:
    void member_func(const string &str){
        cout<<str<<endl;
    }
};

class FANHANSHU{
    public:
    void operator()(const string &str){
        cout<<str<<endl;
    }
};

int main()
{
    PrintStr ps;
    auto lambda_func = [](const string &str){cout<<str<<endl;};
    function<void (const string &)> s1 = free_func;
    function<void (const string &)> s2 = lambda_func;
    function<void (const string &)> s3 = FANHANSHU();
    function<void (const string &)> s4 = bind(&PrintStr::member_func, &ps, placeholders::_1);
    s1("wht");
    s2("wht");
    s3("wht");
    s4("wht");
    return 0;
}