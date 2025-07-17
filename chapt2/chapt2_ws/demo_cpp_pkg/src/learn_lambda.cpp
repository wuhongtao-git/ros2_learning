#include <iostream>
#include <algorithm>

using namespace std;

int main()
{
    auto add = [](int a, int b)
    { return a + b; };
    int sum = add(3, 5);
    auto printsum = [sum]()
    { cout << "3 + 5 = " << sum << endl; };
    printsum();
    return 0;
}