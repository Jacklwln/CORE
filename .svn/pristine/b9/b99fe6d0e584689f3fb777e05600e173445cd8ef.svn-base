/* ************************************************
   File: dagstuhl-2017.cpp

   Purpose:  Some examples arising at the
   			Dagstuhl Seminar 17481
   		"Reliable Computation and Complexity on the Reals"
			Nov 26-Dec 1, 2017.

   Usage:
        % dagstuhl-2017		-- non-verbose output
        % dagstuhl-2017 1	-- verbose output

   Author: Chee Yap

   Since Core Library 2.0
   $Id: testBigFloat.cpp,v 1.3 2010/06/25 09:04:03 exact Exp $
 ************************************************ */  

#ifndef CORE_LEVEL
#   define CORE_LEVEL 3
#endif

#include "CORE.h"

using namespace std;
using namespace CORE;

  // Define "erf" function to be "min" (hack)
  Expr erf(Expr a, Expr b){
      if (a>b) return b;
      return a;
  };

  // logistic function
  Expr logistic(int n){
      Expr x = Expr("1/2");
      Expr c = Expr("375/100");
      for (int i=0; i< n; i++){
	  x = c*x*(1-x);
	  cout << "x[" << i << "] = " << x << endl;
      }
      return x;
  }

//////////////////////////////////////////////////
// main
//////////////////////////////////////////////////
int main( int argc, char *argv[] ) {


  // Command line arguments:
  if (argc>1) coretest_verbose = true;
  int prec = 52;
  if (argc>2) prec = atoi(argv[2]);	// prec is relative precision
  
  // Global unit test variables:
  setDefaultPrecision(prec, CORE_INFTY);
  cout.precision(20);	
  coretest_error=false;
  coretest_verbose=false;

  //////////////////////////////////////////////////
  // Evaluating Rump's Polynomial (Dagstuhl talk of Paul Zimmermann)
  //////////////////////////////////////////////////

  cout <<  "========================================\n"
      << "(Example 1) Evaluating Rump's Polynomial \n"
      << "       poly = 21*bb - 2*aa + 55*bb*bb - 10*aa*bb + (a/(2*b));\n"
      << " where \n"
      << "       a=77617, b=33096, aa=a^2,  bb=b^2. \n"
      <<  "========================================"
      << endl;

  BigFloat a("77617");
  BigFloat b("33096");
  BigFloat aa = a*a;
  BigFloat bb = b*b;

  core_test(aa, a*a, "aa is wrong");

  BigFloat poly =  21*bb - 2*aa + 55*bb*bb - 10*aa*bb + (a/(2*b));

  core_test(poly, poly, "poly is wrong");

  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
  // Norbert Mueller's Logistic Map Example at Dagstuhl talk
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
  
  cout <<  "========================================\n"
       << "(Example 2) Computing the logistic map\n" 
       <<  "            x' = c*x*(1-x)   where c = 1/2, x=3.75 \n"
       <<  "========================================"
       << endl;

  logistic(10);
  
  
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
  // Siegfried Rump's Example at Dagstuhl talk
  //%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%/
  //
  // 		erf(min(1000, abs(R, sin(4* atan(1)))))
  //
  // 			-- but I do not have "erf"
  //
  // 	where R is very large.
  //
  // 	Note that atan(1)=Pi/4, and so sin(4*atan(1))= sin(Pi)=0.
  // 		Thus ans should be 0.   Thus erf(0)=0.
  // 		Since Core Library does not have erf(x),
  // 		we just compute "min"
  
  cout <<  "========================================\n"
       << "(Example 3) Computing Rump's Example \n"
      << "      ans = erf(Expr(1000), abs(R * sin(4* atan(1)))) \n"
      << " where erf is redefined as the min(a,b).\n"
      <<  "========================================"
      << endl;

  Expr R=1000000000000;
  Expr ans = erf(Expr(1000), abs(R * sin(4* atan(1))));

  core_test(1000, ans, "ans is wrong");



  return 0;
}//main


/* **************************************** ignore!

  // set Default Input Digits (for conversion to internal format):
  // 	defInputDigits = CORE_INFTY;	// exact input
  // 	defInputDigits = 20;		// round to 20 digits

  // setDefaultPrecision (R,A) -- sets "mixed R relative bits, A absolute bits"
  // To get either unmixed R relative bits, set A=CORE_INFTY
  // To get either unmixed A absolute bits, set R=CORE_INFTY
		setDefaultPrecision(160, CORE_INFTY);
	
  // Set output (or print) precision 
  		//setDefaultPrintDigits(67);	// 67 bits is 20 digits
  		cout.precision(20);	// 67 bits is 20 digits
  // To get scientific format:
  	cout.setf(ios::scientific, ios::floatfield);
  // To get positional format:
	setPositionalFormat();
  // To get back default format:
  //	cout.setf(0, ios::floatfield); 


  			// REMARK: this is really only a convention.
  f = BigFloat("1"); 		// f=1
  core_test(1, f.get_exp(), "f.get_exp() is wrong");
  f = BigFloat("0.5"); 		// f=1/2
  core_test(0, f.get_exp(), "f.get_exp() is wrong");
  f = BigFloat("0.125"); 	// f=1/8
  core_test(-2, f.get_exp(), "f.get_exp() is wrong");
  f = BigFloat("1.125");	 // f=9/8
  core_test(1, f.get_exp(), "f.get_exp() is wrong");
  
  // get mantissa:
  f = BigFloat("31459");
  BigInt z; int exp;
  exp = f.get_z_exp(z);
  
  if(exp >= 0)
    z *= power(BigInt(2), exp);
  else
    z /= power(BigInt(2), -exp);

  core_test("31459", z.get_str(), "f.get_z_exp() is wrong");
  
  
  // get exponent:
  BigFloat f("0"); 		// f=0
  core_test(0, f.get_exp(), "f.get_exp() is wrong");
  			// REMARK: this is really only a convention.
  f = BigFloat("1"); 		// f=1
  core_test(1, f.get_exp(), "f.get_exp() is wrong");
  f = BigFloat("0.5"); 		// f=1/2
  core_test(0, f.get_exp(), "f.get_exp() is wrong");
  f = BigFloat("0.125"); 	// f=1/8
  core_test(-2, f.get_exp(), "f.get_exp() is wrong");
  f = BigFloat("1.125");	 // f=9/8
  core_test(1, f.get_exp(), "f.get_exp() is wrong");
  
  // get mantissa:
  f = BigFloat("31459");
  BigInt z; int exp;
  exp = f.get_z_exp(z);
  
  if(exp >= 0)
    z *= power(BigInt(2), exp);
  else
    z /= power(BigInt(2), -exp);

  core_test("31459", z.get_str(), "f.get_z_exp() is wrong");
  //
  
  // BigFloat2 string input:
  BigFloat2 f2("0.0007891", 10); 
  core_test(".0007891", f2.get_str(7), "BigFloat string input is wrong");
  //
  
  f = "0.0625";
  f2 = BigFloat2(f);
  core_test(".0625", f2.get_str(), "BigFloat2.get_str is wrong");

  if (coretest_error == false)
    cout << "CORRECT!!! all test are passed" << endl;

  return 0;
}//main
 **************************************** */
