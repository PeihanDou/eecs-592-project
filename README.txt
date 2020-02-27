mysearch.py

Change The domain to perform different problems.
In mysearch.py, one only need to uncomment lines to make the code solve different problems.

Fisrt make sure the variable 'DEBUG' is False.
Then according to the problem you want to  test, uncomment the corresponding open file command. Then run the code. You will see the result in the terminal.

-----------------------------------
For example:
If you want to solve route plannning problem, you can uncomment the line:
#test = 'route.txt
and make sure route.txt have the following format:
[Start City]
[Destination City]
[Searching Method]

All cities should be in the map given by the project.
For instance:
Ann Anbor
Romeo
D
The code will search a path from Ann Arbor to Romeo using DFS. 
The result will be: [(path,cost),total expand nodes number]
---------------------------------------
[('Ann Arbor->Romulus->Detroit->Farmington Hills->Pontiac->Romeo', 125.7), 19]
---------------------------------------

If you want to solve tsp problem, you can uncomment the line:
#test = 'tsp.txt'
and make sure tsp.txt have the following format:
[Start City]
[Searching Method]

All cities should be in the map given by the project.
For instance:
Pontiac
A
The code will search a solution start from Pontiac using A* search. 
The result will be: [(path,cost),total expand nodes number]
---------------------------------------
Travel Salesman Problem:
[('Ann Arbor->BrightonAnn Arbor->Romulus->Detroit->Plymouth->Farmington Hills->Royal Oak->Pontiac->Sterling Heights->Romeo', 198.3), 6711]
----------------------------------------