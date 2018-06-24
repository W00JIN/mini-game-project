#include <iostream>
#include <vector>
#include <stdio.h>

using namespace std;

int main(void)
{
	//freopen("input.txt", "r", stdin);
	int w, h, n;
	int i, j;
	vector<int> arr;
	
	scanf("%d %d", &w, &h);
	w /= 3;
	
	for(i = 0; i < h; i++)
    {
		for(j = 0; j < w; j++)
        {
			scanf("%3d", &n);
			switch(n)
            {
                case 0: arr.push_back(0); break;
                case 1: arr.push_back(3); break;
                case 10: arr.push_back(28); break;
                case 11: arr.push_back(31); break;
                case 100: arr.push_back(224); break;
                case 101: arr.push_back(227); break;
                case 110: arr.push_back(252); break;
                case 111: arr.push_back(255); break;
			}
		}
	}

	for(i = 0; i < h; i++)
    {
		for(j = 0; j < w; j++)
        {
			printf("st7586_write(ST_DATA,0x%02x);\n", arr[i*w + j]);
		}
		printf("\n");
	}
}