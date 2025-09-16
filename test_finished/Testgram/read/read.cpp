/**
 * @author Wu Jia
 * @brief   �������ļ�
 *          ûɶ�������
 */
#include <iostream>
#include <string>
#include <regex>
#include <vector>
#include <iomanip>

using namespace std;

vector<string> extractCoefficients(const string& expr) 
{
    vector<string> coeffs;
    
    
    regex rx3(R"(([-+]?\d+\.\d+)x?)");  
    regex rx2(R"(([-+]?\d+\.\d+)x?)");  
    regex rx1(R"(([-+]?\d+\.\d+)x)");   
    regex rx0(R"(([-+]?\d+\.\d+)$)");   

    smatch match;
    

    if (regex_search(expr, match, rx3)) 
        coeffs.push_back(match[1].str());
    
    

    if (regex_search(expr, match, rx2)) 
        coeffs.push_back(match[1].str());
    
    

    if (regex_search(expr, match, rx1)) 
        coeffs.push_back(match[1].str());
    
    

    if (regex_search(expr, match, rx0)) 
        coeffs.push_back(match[1].str());
    
    
    return coeffs;
}

int main() {
    string line;
    vector<vector<string>> allCoeffs;
    
    
    while (getline(cin, line)) 
    {
        // ���Ұ�������ʽ����
        size_t pos = line.find("f(x) = ");
        if (pos != string::npos) 
        {
            
            string expr = line.substr(pos + 8);
            
            
            size_t plusMinus;
            while ((plusMinus = expr.find("+ -")) != string::npos) 
                expr.replace(plusMinus, 3, "-");
            
            
            // ��ȡϵ��
            vector<string> coeffs = extractCoefficients(expr);
            if (coeffs.size() == 4)   
                allCoeffs.push_back(coeffs);
            
        }
    }
    
    
    for (size_t i = 0; i < allCoeffs.size(); ++i) 
    {
        const auto& coeffs = allCoeffs[i];
        
        
        cout << "{";
        for (size_t j = 0; j < coeffs.size(); ++j) 
        {
      
            if (coeffs[j].front() == '-')
                cout << setw(11) << coeffs[j] << "f";
             
            else
                cout << " " << setw(10) << coeffs[j] << "f";  // ����ǰ���ո����
            
            if (j != 3) 
                cout << ", ";
            
        }
        cout << "}";

        if (i != allCoeffs.size() - 1) 
            cout << ",//";
        else 
            cout << ";//";
        
        cout << endl;
        

        if (i != allCoeffs.size() - 1) 
            cout << "    ";
        
    }
    
    return 0;
}