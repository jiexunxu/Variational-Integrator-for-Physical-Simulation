#include <vector>

using namespace std;

class SparseMatrix{
public:
	int N;
	vector<double> vals;
	vector<int> cols;
	vector<int> rows;

	void set(int row, int col, double val)
	{
		cols.push_back(col);
		rows.push_back(row);
		vals.push_back(val);
	}

	// y=M*b
	void multiply(const vector<double> &b, vector<double> &y)
	{
		if((y.size()!=N)||(b.size()!=N)){
			return;
		}
		for(int i=0;i<N;i++)
			y[i]=0;

		for(unsigned int i=0;i<vals.size();i++)
		{
			int row=rows[i];int col=cols[i];double val=vals[i];
			y[row]=y[row]+val*b[col];
		}
	}
};