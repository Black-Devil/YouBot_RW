Eigene Module

Die beiden folgenden Funktionen fib(), die den n-ten Fibonacci-Wert zurückliefert, und die Funktion fiblist() werden in einer Datei fibonacci.py gespeichert.

def fib(n):
    a, b = 0, 1
    for i in range(n):
        a, b = b, a + b
    return a

def fiblist(n):
    fib = [0,1]
    for i in range(1,n):
        fib += [fib[-1]+fib[-2]]
    return fib

Von einem anderen Programm oder von der interaktiven Shell kann man nun, falls fibonacci.py innerhalb des Suchpfades zu finden ist, die Datei mit den beiden Fibonacci-Funktionen als Modul aufrufen.

>>> import fibonacci
>>> fibonacci.fib(10)
55
>>> fibonacci.fiblist(10)
[0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
>>> fibonacci.__name__
'fibonacci'
>>> 

Pakete

Python ermöglicht es, dass man mehrere Module in einem Paket kapseln kann. Ein Paket kann beliebig viele weitere Pakete enthalten.

Um ein Paket zu erstellen, muss man lediglich einen Unterordner erzeugen, in dem sich eine Datei mit dem Namen __init__.py befinden muss.

Die Datei kann leer sein oder Initialisierungscode in Python enthalten, der beim Import des Paketes einmalig ausgeführt wird.
