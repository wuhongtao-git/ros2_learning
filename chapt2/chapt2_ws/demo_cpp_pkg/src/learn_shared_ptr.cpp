#include <iostream>
#include <memory>

using namespace std;

int main()
{
    auto p1 = make_shared<string>("this is a str.");
    cout << p1.use_count() << " " << p1.get() << endl;
    auto p2 = p1;
    cout << p1.use_count() << " " << p1.get() << endl;
    cout << p2.use_count() << " " << p2.get() << endl;
    p1.reset();
    cout << p1.use_count() << " " << p1.get() << endl;
    cout << p2.use_count() << " " << p2.get() << endl;
    cout << p2->c_str() << endl;
    return 0;
}