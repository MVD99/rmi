/*
    This file is part of ciberRatoToolsSrc.

    Copyright (C) 2001-2015 Universidade de Aveiro

    ciberRatoToolsSrc is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    ciberRatoToolsSrc is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
//escolher tipo de mapa  - "X - | + \s"
//mapear - sensores detetar paredes
//guardar - guardar coordenadas das paredes ?
//calcular next

import java.beans.PersistenceDelegate;
import java.io.*;
import java.net.*;
import java.util.*;
import java.util.Vector;

import org.xml.sax.*;
import org.xml.sax.helpers.DefaultHandler;
import javax.xml.parsers.SAXParserFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.parsers.SAXParser;


import ciberIF.*;

/**
 *  the map
 */
class Map {
    static final int CELLROWS = 7;
    static final int CELLCOLS = 14;

    /*! In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
     *  to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
     */
	char[][] labMap;

    public Map()
    {
	    labMap = new char[CELLROWS*2-1][CELLCOLS*2-1];

        for(int r=0; r < labMap.length; r++) {
            Arrays.fill(labMap[r],' ');
        }
    }

};

/**
 *  class MapHandler parses a XML file defining the labyrinth
 */
class MapHandler extends DefaultHandler {

	/**
	 */
	private Map map;

	/**
	 * returns the Parameters collected during parsing of message
	 */
	Map getMap()
	{
		return map;
	}

	public void startElement(String namespaceURI,
	                         String sName, // simple name
	                         String qName, // qualified name
	                         Attributes attrs)
	throws SAXException
	{

	    //Create map object to hold map
	    if(map == null) map = new Map();

		if(qName.equals("Row")) {  // Row Values

            if (attrs != null) {
                String rowStr=attrs.getValue("Pos");
                if(rowStr!=null) {
                    int row = Integer.valueOf(rowStr).intValue();
		            String pattern = attrs.getValue("Pattern");
                    for(int col=0; col < pattern.length(); col++) {
                       if(row % 2 == 0) { // only vertical walls are allowed here
                            if(pattern.charAt(col)=='|') {
                               map.labMap[row][(col+1)/3*2-1] = '|';
                            }
                       }
                       else {// only horizontal walls are allowed at odd rows
                           if(col % 3 == 0) { // if there is a wall at this collumn then there must also be a wall in the next one
                               if(pattern.charAt(col)=='-') {
                                  map.labMap[row][col/3*2] = '-';
                               }
                           }
                       }
                    }
                }
            }
        }
	}

	public void endElement(String namespaceURI,
			        String sName, // simple name
			        String qName  // qualified name
						        )
	throws SAXException
	{
	}
};


/**
 * example of a basic agent
 * implemented using the java interface library.
 */
public class jClientC2 {

    ciberIF cif;
    Map map;
    static String mapName = new String();
    public static void setMap(String a){
        mapName = a;
    }
    enum State {GA, RL, RR, INV, END}

    public static void main(String[] args) throws IOException{

        String host, robName;
        int pos;
        int arg;
        Map map;


        //default values
        host = "localhost";
        robName = "jClientC2";
        pos = 1;
        map = null;

        // parse command-line arguments
        try {
            arg = 0;
            while (arg<args.length) {
                if(args[arg].equals("--pos") || args[arg].equals("-p")) {
                        if(args.length > arg+1) {
                                pos = Integer.valueOf(args[arg+1]).intValue();
                                arg += 2;
                        }
                }
                else if(args[arg].equals("--robname") || args[arg].equals("-r")) {
                        if(args.length > arg+1) {
                                robName = args[arg+1];
                                arg += 2;
                        }
                }
                else if(args[arg].equals("--host") || args[arg].equals("-h")) {
                        if(args.length > arg+1) {
                                host = args[arg+1];
                                arg += 2;
                        }
                }
                else if(args[arg].equals("--map") || args[arg].equals("-m")) {
                        if(args.length > arg+1) {

                                MapHandler mapHandler = new MapHandler();

                                SAXParserFactory factory = SAXParserFactory.newInstance();
                                SAXParser saxParser = factory.newSAXParser();
                                FileInputStream fstream=new FileInputStream(args[arg+1]);
                                saxParser.parse( fstream, mapHandler );

                                map = mapHandler.getMap();

                                arg += 2;
                        }
                }
                else if(args[arg].equals("--mapFile") || args[arg].equals("-mf")) {
                    if(args.length > arg+1) {
                        jClientC2.setMap(args[arg+1]);
                        arg += 2;
                    }
                }
                else throw new Exception();
            }
        }
        catch (Exception e) {
                print_usage();
                return;
        }

        // create client
        jClientC2 client = new jClientC2();

        client.robName = robName;
        double[] angles = {0,90,-90,180};
        // register robot in simulator
        client.cif.InitRobot2(robName, pos, angles, host);
        client.map = map;
        client.printMap();

        // main loop
        client.mainLoop();

    }

    // Constructor
    jClientC2() {
            cif = new ciberIF();
            beacon = new beaconMeasure();

            beaconToFollow = 0;
            ground=-1;
    }

    /**
     * reads a new message, decides what to do and sends action to simulator
     */
    public void mainLoop () throws IOException{ //ver aqui.....
        cif.ReadSensors();
        x0=cif.GetX();
        y0=cif.GetY();
        init=true;
        while(true) {
                cif.ReadSensors(); // ler os sensores .....
                //criar as variaveis....

                decide();
        }
    }


     /*! In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
     *  to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
     */

    /**
     * basic reactive decision algorithm, decides action based on current sensor values
     */
    public void decide() throws IOException{ // num ciclo decide o que vai fazer -> drivemotors
            if(cif.IsObstacleReady(0))
                    irSensor0 = cif.GetObstacleSensor(0);
            if(cif.IsObstacleReady(1))
                    irSensor1 = cif.GetObstacleSensor(1);
            if(cif.IsObstacleReady(2))
                    irSensor2 = cif.GetObstacleSensor(2);
            if(cif.IsObstacleReady(3))
                    irSensor3 = cif.GetObstacleSensor(3);
            if(cif.IsCompassReady())
                    compass = cif.GetCompassSensor();
            if(cif.IsGroundReady())
                    ground = cif.GetGroundSensor(); // ver se esta num dos alvos
            if(cif.IsGPSReady()){
                    x = cif.GetX() - x0;//+-22
                    y = cif.GetY() - y0;//+-13
            }

            if(cif.IsBeaconReady(beaconToFollow))
                    beacon = cif.GetBeaconSensor(beaconToFollow);

            System.out.println("Measures: ir0=" + irSensor0 + " ir1=" + irSensor1 + " ir2=" + irSensor2 + " ir3="+ irSensor3+"\n" + "bussola=" + compass + " GPS-X=" + x + " GPS-y=" + y  +"\n");


            //funcao geral: detetar se está no centro, se sim corre mapping e calcular next, se nao manda andar para onde é preciso
            //andar implica ou curvas ou ahead
            state = Estados();

            switch(state) { /////é aqui que mexemos

                case GA: // andar para a frente
                    if(targetReached()){
                       
                        mappingDecode(); //vai darnos para qual estado ir... aproveitar alguns dos ifs que tinhamos
                    }
                    else{
                        goAhead(); //andar -> funcao de andar
                        //aumenta custo de 2
                       /* int i = position_custo()
                        if (i>=0){tree.cost.get(i)+=1;}
                        else{System.out.print("ES UMA ALEIJADA");} */
                        break;
                    }
                case RL:
                    if(targetReached()){ //se nao atualizar os next
                        mappingDecode();
                        break; //se estivermos no meio da funcao
                    }
                    else{
                        goLeft(); //esquerda -> funcao de rodar
                        //aumenta custo de 1
                       /* int i = position_custo()
                        if (i>=0){tree.cost.get(i)+=1;}
                        else{System.out.print("ES UMA ALEIJADA");}*/
                        break;
                    }

                case RR:
                    if(targetReached()){ //se nao atualizar os next
                        mappingDecode();
                        break; //se estivermos no meio da funcao
                    }
                    else{
                        goRight(); //rodar direita -> funcao de rodar
                        //aumenta custo de 1
                        /*int i = position_custo()
                        if (i>=0){tree.cost.get(i)+=1;}
                        else{System.out.print("ES UMA ALEIJADA");}*/
                        break;
                    }
                case INV: // quando ele tievr que inverter
                    //ele roda uma vez e depois o estado muda para o rr automaticamente e acaba a inversao
                    goRight();
                    //aumenta custo de 1
                    /*int i = position_custo()
                    if (i>=0){tree.cost.get(i)+=1;}
                    else{System.out.print("ES UMA ALEIJADA");}*/

                    break;

                case END:
                    cif.DriveMotors(0.0,0.0);
                    break;
            }
            return;
    }

    public State Estados() throws IOException{
        if(init==true){
            init=false;
            fillMap(); //preencher arrays de coords com " "
            writeMap(); //inicializar o ficheiro mapa
            return State.GA;
        }
        if((compass_goal!=compass) && Math.abs(compass-compass_goal)>=10){ //VERIFICAR SE 10º É MUITO
            if(Math.abs(compass-compass_goal)>100) return State.INV;
            if(Eixo() && nivel() ==-1){ // se o robo estiver a apontar para a nossa esquerda
                if(compass_goal==-90) return State.RL;
                else return State.RR;
            }
            if(compass_goal>compass) return State.RL;
            else return State.RR;
        }else{
            return State.GA;
        }
    }

    //da nos a posicao onde colocar o custo no array -> associa-lo a cada no (a cada posicao)
    public int position_custo(){
        for (int i  =0;i<tree.children.length();i++){
            if (tree.children.get(i).getX()==x && tree.children.get(i).getY()==y){
                return i;
            }
        }
        return -1;
    }

    public boolean targetReached(){ //chegou ao objetivo?
        if (Math.abs(x-next.getX())<=0.2 && Math.abs(y-next.getY())<=0.2){
            return true;
        }else{
            return false;
        }

    }

    public void goLeft(){
        double deltaC = Math.abs(Math.abs(compass_goal)-Math.abs(compass));
        double rot = 0.5 * deltaC; //
        double l = (-rot/2);
        double r = (rot/2);
        cif.DriveMotors(l, r);

    }

    public void goRight(){
        double deltaC = Math.abs(Math.abs(compass_goal)-Math.abs(compass));
        double rot = 0.5 * deltaC; //
        double l = (rot/2);
        double r = (-rot/2);
        cif.DriveMotors(l, r);
    }

    public void goAhead(){ //yr -> y inicial -> funcao de andar para a frente
        double deltaY;
        double deltaX;
        if(Eixo()){ // ele esta virado na horizontal
            deltaY = next.getY() - y; //Y que quero alcançar - y atual
            deltaX = next.getX() - x;
        }
        else{ //robo no eixo veritcal
            deltaY = next.getY() - y; //Y que quero alcançar - y atual
            deltaX = next.getX() - x; //X que quero alcançar - x atual
        }
            double lin = 0.5 * deltaX * nivel(); //nivel corresponde a ser + ou - no eixo
            double rot = 0.5 * deltaY * nivel(); //
            double l = (lin+rot/2);
            double r = (lin-rot/2);
        cif.DriveMotors(l, r);
    }


    //para nao ir para cima quando esta a rodado para baixo ou para a esquerda (-90 e 180)
    public double nivel(){ //alinhador dos goAheads positivo(1) ou negativo(-1)
        if(compass>-45 && compass<135) return 1;
        return -1;
    }
    public boolean Eixo(){ //true se for horizontal
        if((compass>-45 && compass<=45) || (compass>= 135 && compass < -135)) return true;
        else return false;
    }


    public void mappingDecode() throws IOException{
        //mapear
        //detetar paredes, se for ou | ou -, se n for entao "x"
        if(ParedeDireita()){
            if(par(coordDir()))
                addToMap(coordDir(),"|");
            else
                addToMap(coordDir(),"-");
        }else{
            addToMap(coordDir(), "X");
          /*  if (coorDir().getY()%2==0){ //se nao for aquele X no meio das paredes
                //v1 -> v2
                //tree.v2.setParent 
                //v2.setParent = v1
            }*/
        }
        if(ParedeEsquerda()){
            if(par(coordEsq()))
                addToMap(coordEsq(),"|");
            else 
                addToMap(coordEsq(),"-");
        }else{
            addToMap(coordEsq(), "X");
        }
        if(ParedeFrente()){
            if(par(coordFrente()))
                addToMap(coordFrente(),"|");
            else
                addToMap(coordFrente(),"-");
        }else{
            addToMap(coordFrente(), "X");
        }
        if(ParedeTras()){
            if(par(coordTras()))
                addToMap(coordTras(),"|");
            else
                addToMap(coordTras(),"-");
        }else{
            addToMap(coordTras(), "X");
        }

//--------------------- calcular a posicao seguinte--------------------------------


        //adicionar posicao atual a posicoes ja visitadas
        //calcular next 
        vetor vatual= new vetor(Math.round(x),Math.round(y));
        coordsAntigas.addLast(vatual);             //adicionar coordenada atual a lista de onde ele ja esteve
        List viz = vizinhos(); // fazer a funcao vizinhos -> devolve as posicoes a volta sem parede
        List pos = possiveis();                    //Tem lista com todas as posicoes possiveis de ir na proximidade
        

        
            
        for (int i = 0; i<viz.size();i++){ //acrescentar os nos filhos possiveis
            tree.addChild(viz); // adiciona os nos possiveis a partir da posicao onde ele esta
            //se for a andar para a frente = custo 2
            // -------------- o compass, o next e o state e no proximo for----- 
            if (viz.get(i) == coordFrente()){ // se estiver no estado de andar para a frente o custo e compasso vao ser...
                tree.cost.get(i)+=2;
                compass_goal=compass;
                //state = State.GA;
            }else if(viz.get(i)==coordEsq()){ // se for rodar
                tree.cost.get(i)+=1;
                if(compass==180){
                    compass=-compass;
                }
                compass_goal=compass+90;
                //state = State.RL;
            }else if(viz.get(i)==coordDir()){ //rodar para a direita
                tree.cost.get(i)+=1;
                compass_goal=Math.abs(compass)-90;
                //state = State.RR;
            }else if(viz.get(i)==coordTras()){ //rodar para tras
                tree.cost.get(i)+=1;
                compass_goal=Math.abs(compass)-180;
                //state = State.INV;
            }else{System.out.print("ES UMA ALEIJADA");}
        }

        // percorrer a arvore e ver em que no ele esta neste momento e ver qual dos filhos desse no tem menor custo
        for(int i = 0;i<tree.size();i++){ //percorrer a arvore ate ao no onde o robo esta
            if(tree.parent.getX()==x && tree.parent.getY()==y){
                // a posicao dos nos dos filhos = a posicao no array do custo
                //vai ver os filhos desse no (posicao deles) e depois ve qual deles tem menor custo -> posicao next :))))
                break;
            }
        }

    /*
     */
        
        //definir compass_goal


        //calcular o estado quando esta no centro da celula -> estado seguinte
        
        
    }

    public void fillMap(){ //chamada no estado init para encher o mapa com " "
        for(int i=0; i<28;i++){
            for (int j=0; j<56; j++){
                coords[i][j]=" ";
                if(i==14 && j==28) coords[i][j]= "I";
            }
        }
    }

    public void addToMap(vetor v, String a) { //escrever no coords a String certa REVER
        //Adicionar verificacao de empty no array
        if(!onMap(v)){
            for(int i=1; i<28;i++){
                for (int j=1; j<56; j++){
                    if(v.getX()==i && v.getY()==j){
                        coords[i][j]= a;
                        break;
                    }
                }
            }
        }
        else ;
    }

    public List<vetor> possiveis(){ // devolve um vetor com os vizinhos onde ele ainda nao esteve
        List r = new List<vetor>();
        //Começar por verificar vizinhos
        if ((coords[coordDir().getX()][coordDir().getY()]=="X") && !coordsAntigas.contains(coordDir())){
            r.add(coordDir());
        }
        if ((coords[coordEsq().getX()][coordEsq().getY()]=="X") && !coordsAntigas.contains(coordEsq())){
            r.add(coordEsq());
        }
        if ((coords[coordFrente().getX()][coordFrente().getY()]=="X") && !coordsAntigas.contains(coordFrente())){
            r.add(coordFrente());
        }
        if ((coords[coordTras().getX()][coordTras().getY()]=="X") && !coordsAntigas.contains(coordTras())){
            r.add(coordTras());
        }
        //verificar o resto do mapa
        //percorre o mapa todo e ve se é X
        vetor l = new vetor();
        for(int i=1; i<28;i++){
            for (int j=1; j<56; j++){
                l.setXY(i,j);
                if((coords[i][j] == "X") && !coordsAntigas.contains(l) && l.getY()%2==0){ // ve quando esta no meio das celulas
                    r.add(l);
                }
            }
        }
        return r;
    }


    public void writeMap() throws IOException{ //Escreve o mapa no file
        File fileMap = new File (mapName);
        Scanner fin = new Scanner (fileMap); //ficheiro de entrada = ficheiro de saida
        if (!fileMap.exists()){
            System.out.println("Ficheiro nao existe");
        }
        PrintWriter write = new PrintWriter(fileMap);
        String a = new String();
        for(int i=1; i<28;i++){
            for (int j=1; j<56; j++){
                a = a+coords[i][j]; // a = linha toda
                if (j==55) a=a+"\n";
            }
            write.print(a);
        }
        fin.close();
        write.close();    
    }

    public boolean onMap(vetor v) throws IOException{  //Verificar se as coordenadas dadas já estão preenchidas
        File fileMap = new File (mapName);
        Scanner fin = new Scanner (fileMap); //ficheiro de entrada = ficheiro de saida
        if (!fileMap.exists()){
            System.out.println("Ficheiro nao existe");
        }
        double linha =0;
        double coluna =0;
        String b = new String();
        while (fin.hasNextLine()){ //vamos ver na linha
            if(linha == v.getY()+14){
                //percorrer nas colunas
                while(fin.hasNext()){
                    if(coluna==v.getX()+28){
                        b=fin.next();
                        if(b.equals("|") || b.equals("X") || b.equals("-") || b.equals("I")){
                            fin.close();
                            return true;
                        } 
                        fin.close();
                        return false;
                    }
                    coluna++;
                }
            }
            linha++;
        }
        fin.close();
        return false;
    }

    public boolean par(vetor v){//impar horizontal, par vertical
        if(v.getX()%2==0) return true;
        return false;
    }

    //ver se existe parede nas 4 direcoes
    public boolean ParedeFrente(){
        if(irSensor0>=2.3) return true;
        return false;
    }
    public boolean ParedeTras(){
        if(irSensor3>=2.3) return true;
        return false;
    }
    public boolean ParedeDireita(){
        if(irSensor2>=2.3) return true;
        return false;
    }
    public boolean ParedeEsquerda(){
        if(irSensor1>=2.3) return true;
        return false;
    }



    static void print_usage() {
             System.out.println("Usage: java jClientC2 [--robname <robname>] [--pos <pos>] [--host <hostname>[:<port>]] [--map <map_filename>]");
    }

    public void printMap() {
           if (map==null) return;

           for (int r=map.labMap.length-1; r>=0 ; r--) {
               System.out.println(map.labMap[r]);
           }
    }


    //AUXILIARES
    public vetor coordEsq(){
        vetor v= new vetor(0,0);
        if(compass==0) v.setXY(x,y+1);
        else if(compass==90) v.setXY(x-1,y);
        else if(compass==-90) v.setXY(x+1,y);
        else v.setXY(x,y-1);
        return v;
    }
    public vetor coordDir(){
        vetor v= new vetor(0,0);
        if(compass==0) v.setXY(x,y-1);
        else if(compass==90) v.setXY(x+1,y);
        else if(compass==-90) v.setXY(x-1,y);
        else v.setXY(x,y+1);
        return v;
    }
    public vetor coordFrente(){
        vetor v= new vetor(0,0);
        if(compass==0) v.setXY(x+1,y);
        else if(compass==90) v.setXY(x,y+1);
        else if(compass==-90) v.setXY(x,y-1);
        else v.setXY(x-1,y);
        return v;
    }
    public vetor coordTras(){
        vetor v= new vetor(0,0);
        if(compass==0)  v.setXY(x-1,y);
        else if(compass==90)  v.setXY(x,y-1);
        else if(compass==-90)  v.setXY(x,y+1);
        else  v.setXY(x+1,y);
        return v;
    }


    /*public int deadlock(){ //return 2 "normal", 3 "beco", 1 ou 0 "cruzamento";
        int c=0;
        if(irSensor0>=2.3) c++; //2.5
        if(irSensor1>=2.3) c++;
        if(irSensor2>=2.3) c++;
        if(irSensor3>=2.3) c++;
        return c;
    }
    */

    //VARIAVEIS

    private String robName;
    private double irSensor0, irSensor1, irSensor2, irSensor3, compass, compass_goal, x, y, x0,y0, custo;
    //sensores, bussola, bussola para o objetivo, coordenadas, coordenadas iniciais
    // compass_goal para onde ele vai ter de rodar
    private beaconMeasure beacon;
    private int ground;
    private boolean collision,init; //initial position?
    private vetor next = new vetor(); //para onde vai a seguir
    private State state;
    private int beaconToFollow;
    private String[][] coords = new String[28][56]; //linhas, colunas
    private LinkedList<vetor> coordsAntigas = new LinkedList<vetor>(); //linhas, colunas
    
    private TreeNode<T> tree = new TreeNode<T>(vetor(x0,y0)); //root -> I

    public class vetor{
        private double x;
        private double y;

        public vetor(){}
        public vetor(double x, double y){
            this.x=x;
            this.y=y;
        }
        public double getX(){
            return x;
        }
        public double getY(){
            return y;
        }
        public void setX(double x){
            this.x=x;
        }
        public void setY(double y){
            this.y=y;
        }
        public void setXY(vetor v){
            this.x=v.getX();
            this.y=v.getY();
        }
        public void setXY(double xf, double yf){
            double nx = Double.valueOf(Math.round(xf));
            double ny = Double.valueOf(Math.round(yf));
            x=nx;
            y=ny;
        }
    }

    // se e grafo ou arvore
    //custo 
    //scanner

};


