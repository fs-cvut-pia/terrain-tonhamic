[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-22041afd0340ce965d47ae6ef1cefeee28c7c493a6346c4f15d667ab976d596c.svg)](https://classroom.github.com/a/ePPRhryX)
# Hledání cesty v terénní mapě

Máte zde připravený základ kódu pro práci s terénní mapou. Ta je reprezentována maticí 256x256 celých čísel představující nadmořskou výšku v metrech (soubor *terrain.dat*).

Najdete zde třídu *TerrainMap*, která obsahuje metody pro načtení mapy ze souboru a přístup k jednotlivým prvkům matice. Souřadnice můžete ukládat do objektů třídy *Point*, které podporují i některé vektorové operace. Dále můžete využít třídu *Matrix< >* pro reprezentaci libovolných pomocných maticových dat.

Dále je zde abstraktní třída *Path* která slouží k ukládání souřadnic jednotlivých bodů cesty do struktury vector<Point>, obsahuje dále metody pro analýzu cesty a výpis cesty do souboru. Předpokládáme, že se můžeme po mapě pohybovat vždy o 1 pole ve vodorovném, svislém a šikmém směru. Vaším úkolem bude vytvořit odvozené třídy s implementovanou metodou *find()* pro nalezení cest následujících typů:

- **"Letadlo"** - nejkratší cesta mezi dvěma body nehledě na nadmořskou výšku
- **"Loď"** - nejkratší cesta pouze po polích, kde výška < 0 (kromě výchozího a cílového bodu)
- **"Silnice"** - nejkratší cesta pouze po polích, kde výška > 0 a zároveň stoupání trasy mezi následujícími body < 6%
- **"Silnice + trajekt"** - nejrychlejší cesta mezi dvěma body
  - pokud je výška > 0, stoupání trasy mezi následujícími body musí být < 6%
  - pokud je výška < 0, stoupání není podstatné, ale rychlost pohybu je pouze čtvrtinová oproti pohybu na souši
- **"Nákladní železnice"** - cesta mezi dvěma body, která minimalizuje celkový součet výškových metrů (stoupání i klesání) na trase a zároveň stoupání trasy mezi následujícími body < 4%

Najděte tyto typy cest mezi body [198 205] a [78 17]

Funkce *main(...)* je vytvořena tak, že program přijímá jako vstup z příkazové řádky název souboru s mapou a souřadnice počátečního a koncového bodu, např.:
 	`./findpath terrain.dat 198 205 78 17`

Skript `./plot_path.py terrain.dat cesta.dat` vykreslí terénní mapu a do ní nalezenou cestu.

Skript `./generate_terrain.py` vygeneruje novou náhodnou terénní mapu. 

Oba skripty vyžadují Python 3 a nainstalované knihovny *numpy, matplotlib*, generátor navíc ještě knihovnu *noise*.
