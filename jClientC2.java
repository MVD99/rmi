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
        end=true;
        fillMap();
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

            if(targetReached()){
                System.out.println("\n Measures: ir0=" + irSensor0 + " ir1=" + irSensor1 + " ir2=" + irSensor2 + " ir3="+ irSensor3+"\n" + "bussola=" + compass + " GPS-X=" + x + " GPS-y=" + y);
                System.out.println("f: "+ParedeFrente()+" esq: "+ParedeEsquerda()+" dir: "+ParedeDireita()+" tras: "+ParedeTras());
                System.out.println("init: "+init+" targetReached? "+targetReached() + "\n");
            }
            //funcao geral: detetar se está no centro, se sim corre mapping e calcular next, se nao manda andar para onde é preciso
            //andar implica ou curvas ou ahead
            

            state = Estados();
            if(next.getX()==6 && next.getY()==2){state = State.END;}
            System.out.println("Estado: "+state);
            switch(state) { /////é aqui que mexemos

                case GA: // andar para a frente
                    if(targetReached()){
                        mappingDecode();
                        cif.DriveMotors(0.0,0.0);
                    }
                    else{
                        goAhead(); //andar -> funcao de andar   
                    }
                    break;

                case RL:
                    if(targetReached()){ //se nao atualizar os next
                        mappingDecode(); //se estivermos no meio da funcao
                        cif.DriveMotors(0.0,0.0);
                    }
                    else{
                        goLeft(); //esquerda -> funcao de rodar
                    }
                    break;

                case RR:
                    if(targetReached()){ //se nao atualizar os next
                        mappingDecode();
                        cif.DriveMotors(0.0,0.0);
                    }
                    else{
                        goRight(); //rodar direita -> funcao de rodar
                    }
                    break;

                case INV: // quando ele tievr que inverter
                    //ele roda uma vez e depois o estado muda para o rr automaticamente e acaba a inversao
                    goInv();
                    
                    break;

                case END:
                    cif.DriveMotors(0.0,0.0);
                    if(end==true){
                        end=false;
                        writeMap();
                    }
                    break;
            }
            return;
    }

    public State Estados() throws IOException{
        if(init==true){
            init=false;
            return State.GA;
        }
        if(endS==true){
            endS=false;        
            return State.END;
        }

        if(Math.abs(Math.abs(compass)-Math.abs(compass_goal))>=15){ //VERIFICAR SE 10º É MUITO
            
            if(Math.abs(Math.abs(compass)-Math.abs(compass_goal))>110) return State.INV;

            if(compass_goal == -90){
                System.out.println("cg>c: " + compass_goal + " " +compass);
                if(compass>-90 && compass<90) return State.RR;
                else return State.RL;
            }
            else if(compass_goal == 180){
                System.out.println("cg>c: " + compass_goal + " " +compass);
                if(compass<0) return State.RR;
                else return State.RL;
            }
            else{
                System.out.println("cg>c: " + compass_goal + " " +compass);
                if(compass_goal>compass) return State.RL;
                else return State.RR;

            }
        }else{
            return State.GA;
        }
    }
    
    public boolean targetReached(){ //chegou ao objetivo?
        if (Math.abs(x-next.getX())<=0.15 && Math.abs(y-next.getY())<=0.15){
            return true;
        }else{
            return false;
        }

    }

    public void goLeft(){
        double deltaC = Math.abs(Math.abs(compass_goal)-Math.abs(compass));
        double rot = 0.5 * deltaC;
        double l = -rot;
        double r = rot;
        cif.DriveMotors(l, r);

    }

    public void goRight(){
        double deltaC = Math.abs(Math.abs(compass_goal)-Math.abs(compass));
        double rot = 0.5 * deltaC;
        double l = rot;
        double r = -rot;
        cif.DriveMotors(l, r);
    }

    public void goInv(){
        //compass_goal = compass - 180;
        cif.DriveMotors(0.15,-0.15);
    }

    public void goAhead(){ //yr -> y inicial -> funcao de andar para a frente
        double deltaY = next.getY() - y; //Erro do Y
        double deltaX = next.getX() - x; //Erro do X
        double l, r;
        double k = 0.1;
       
        if(Eixo()){ // ele esta virado na horizontal
            l =0.1 - k * deltaY * nivel(); //nivel corresponde a ser + ou - no eixo
            r =0.1 - k * -deltaY * nivel(); // Valor maximo de velocidade menos 
        }
        else{ //robo no eixo veritcal
            l =0.1 - k * -deltaX * nivel(); //nivel corresponde a ser + ou - no eixo
            r =0.1 - k * deltaX * nivel(); //
        }
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
        Set<vetor> localViz = new HashSet<vetor>();
        //adicionar coordenada atual a lista de coordendas visitadas
        vetor abc = new vetor();
        abc.setXY(next);
        System.out.println("Coords Antigas:");
        coordsAntigas.add(abc);
        Iterator<vetor> it2 = coordsAntigas.iterator();
            while(it2.hasNext()){
                it2.next().printV();
            }
        System.out.println("Size - " + coordsAntigas.size());

        Iterator<vetor> rv = visitaveis.iterator();
        while(rv.hasNext()){
            if(rv.next().equals(abc)) rv.remove();
        }
        //MAPEAR
        //detetar paredes, se for ou | ou -, se n for entao "x"
        //se coordenada nao for parede e nao tiver sido visitada e nao estiver nos visitaveis, adiciona aos visitaveis
        vetor pf = new vetor();
        vetor pe = new vetor();
        vetor pd = new vetor();
        vetor pt = new vetor();

        if(ParedeDireita()){
            if(!par(coordDir()))
                addToMap(coordDir(),"|");
            else
                addToMap(coordDir(),"-");
        }else{
            addToMap(coordDir(), "X");
            addToMap(coord2Dir(), "X");
            pd.setXY(coord2Dir());
            if(!coordsAntigas.contains(pd)){
                localViz.add(pd);
                visitaveis.add(pd);
            }
        }
        if(ParedeEsquerda()){
            if(!par(coordEsq()))
                addToMap(coordEsq(),"|");
            else 
                addToMap(coordEsq(),"-");
        }else{
            addToMap(coordEsq(), "X");
            addToMap(coord2Esq(), "X");
            pe.setXY(coord2Esq());
            if(!coordsAntigas.contains(pe)) {
                localViz.add(pe);
                visitaveis.add(pe);
            }
        }
        if(ParedeFrente()){
            if(!par(coordFrente()))
                addToMap(coordFrente(),"|");
            else
                addToMap(coordFrente(),"-");
        }else{
            addToMap(coordFrente(), "X");
            addToMap(coord2Frente(), "X");
            pf.setXY(coord2Frente());
            if(!coordsAntigas.contains(pf)){
                localViz.add(pf);
                visitaveis.add(pf);
            }
        }
        if(ParedeTras()){
            if(!par(coordTras()))
                addToMap(coordTras(),"|");
            else
                addToMap(coordTras(),"-");
        }else{
            addToMap(coordTras(), "X");
            addToMap(coord2Tras(), "X");
            pt.setXY(coord2Tras());
            if(!coordsAntigas.contains(pt)){
                visitaveis.add(pt);
            }
        }

        System.out.println("Local vizinhos:");
        Iterator<vetor> it3 = localViz.iterator();
            while(it3.hasNext()){
                it3.next().printV();
            }
        System.out.println("Size - " + localViz.size());
        System.out.println("Global visitaveis:");
        Iterator<vetor> it4 = visitaveis.iterator();
            while(it4.hasNext()){
                it4.next().printV();
            }
        System.out.println("Size - " + visitaveis.size());

        if(visitaveis.isEmpty()){endS=true;} // quando visitar tudo ou se o clock estiver quase acabar
        
//--------------------- calcular a posicao seguinte--------------------------------
        else{        
            if(localViz.size()==0) next.setXY(visitaveis.iterator().next());
            else{
                next.setXY(localViz.iterator().next());
                if(next.getX()==coord2Frente().getX() && next.getY()==coord2Frente().getY()) compass_goal=compass;
                if(next.getX()==coord2Dir().getX() && next.getY()==coord2Dir().getY()) compass_goal=compass-90;
                if(next.getX()==coord2Esq().getX() && next.getY()==coord2Esq().getY()) compass_goal=compass+90;
                if(next.getX()==coord2Tras().getX() && next.getY()==coord2Tras().getY()) compass_goal=compass-180;
            }
            arrendAngulo();
        }
        //System.out.println("Compass_goal antes do ArrendAngulo: "+ compass_goal);
        System.out.println("Mapping decode next x= "+next.getX()+" y= "+next.getY()+" compass_goal= "+compass_goal + " compass= " + compass);          
    }

    public LinkedList<vetor> vizinhos(){
        LinkedList <vetor> viz = new LinkedList<>();
        if(!ParedeFrente()){ viz.add(coord2Frente()); }
        if(!ParedeDireita()){ viz.add(coord2Dir()); }
        if(!ParedeEsquerda()){ viz.add(coord2Esq()); }
        if(!ParedeTras()){ viz.add(coord2Tras()); }
        return viz;
    }

//------------------------------------- MAPA MAPA MAPA -------------------------------------
    public void fillMap(){ //chamada no estado init para encher o ARRAY do mapa com " "
        for(int i=0; i<28;i++){ //percorremos as linhas
            for (int j=0; j<56; j++){ //percorremos as colunas para cada linha
                coords[i][j]=" ";
                if(i==14 && j==28) coords[i][j]= "I";
            }
        }
    }

    public void addToMap(vetor v, String a) throws IOException { //escrever no coords a String certa REVER
        int col = (int) (v.getX())+28;
        int lin = 14 - (int) (v.getY());
        if(!onMap(v))coords[lin][col]= a;
    }


    public void writeMap() throws IOException{ //Escreve o mapa no file
        File fileMap = new File (mapName);
        Scanner fin = new Scanner (fileMap);
        if (fileMap.createNewFile()) {
            System.out.println("File created: " + fileMap.getName());
        } else {
            System.out.println("File already exists. Cleaning and rewriting the file.");
        }
        FileWriter writeFile = new FileWriter(fileMap);
        String a = new String();
        for(int lin=1; lin<28;lin++){
            a="";
            for (int col=1; col<56; col++){
                a = a + coords[lin][col]; // a = linha toda
                if (col==55) a = a + "\n";
            }
            writeFile.write(a);
        }
        fin.close();
        writeFile.close();    
    }

    public boolean onMap(vetor v) {  //Verificar se as coordenadas dadas já estão preenchidas
        int col = (int) (v.getX())+28;
        int lin = 14 - (int) (v.getY());
        if(coords[lin][col].equals("|") || coords[lin][col].equals("X") || coords[lin][col].equals("-") || coords[lin][col].equals("I")){
            return true;
        } 
        return false;
    }

    public boolean par(vetor v){//impar horizontal, par vertical
        if(v.getX()%2==0) return true;
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
    //ver se existe parede nas 4 direcoes
    public boolean ParedeFrente(){
        if(irSensor0>=1.2) return true;
        return false;
    }
    public boolean ParedeTras(){
        if(irSensor3>=1.2) return true;
        return false;
    }
    public boolean ParedeDireita(){
        if(irSensor2>=1.2) return true;
        return false;
    }
    public boolean ParedeEsquerda(){
        if(irSensor1>=1.2) return true;
        return false;
    }

    //Dar coordenadas
    public vetor coordEsq(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x,y+1);
        else if(compass>=45 && compass <135) v.setXY(x-1,y);
        else if(compass==-90) v.setXY(x+1,y);
        else v.setXY(x,y-1);
        return v;
    }
    public vetor coordDir(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x,y-1);
        else if(compass>=45 && compass <135) v.setXY(x+1,y);
        else if(compass<-45 && compass >= -135) v.setXY(x-1,y);
        else v.setXY(x,y+1);
        return v;
    }
    public vetor coordFrente(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x+1,y);
        else if(compass>=45 && compass <135) v.setXY(x,y+1);
        else if(compass<-45 && compass >= -135) v.setXY(x,y-1);
        else v.setXY(x-1,y);
        return v;
    }
    public vetor coordTras(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45)  v.setXY(x-1,y);
        else if(compass>=45 && compass <135)  v.setXY(x,y-1);
        else if(compass<-45 && compass >= -135)  v.setXY(x,y+1);
        else  v.setXY(x+1,y);
        return v;
    }

    //2 coordenadas -> centro da celula
    public vetor coord2Esq(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x,y+2);
        else if(compass>=45 && compass <135) v.setXY(x-2,y);
        else if(compass<-45 && compass >= -135) v.setXY(x+2,y);
        else v.setXY(x,y-2);
        return v;
    }
    public vetor coord2Dir(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x,y-2);
        else if(compass>=45 && compass <135) v.setXY(x+2,y);
        else if(compass<-45 && compass >= -135) v.setXY(x-2,y);
        else v.setXY(x,y+2);
        return v;
    }
    public vetor coord2Frente(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45) v.setXY(x+2,y);
        else if(compass>=45 && compass <135) v.setXY(x,y+2);
        else if(compass<-45 && compass >= -135) v.setXY(x,y-2);
        else v.setXY(x-2,y);
        return v;
    }
    public vetor coord2Tras(){
        vetor v= new vetor(0,0);
        if(compass<45 && compass >=-45)  v.setXY(x-2,y);
        else if(compass>=45 && compass <135)  v.setXY(x,y-2);
        else if(compass<-45 && compass >= -135)  v.setXY(x,y+2);
        else  v.setXY(x+2,y);
        return v;
    }

    public void arrendAngulo(){
        if(compass_goal<45 && compass_goal >=-45)  compass_goal=0;
        else if(compass_goal>=45 && compass_goal <135)  compass_goal=90;
        else if(compass_goal<-45 && compass_goal >= -135)  compass_goal=-90;
        else compass_goal=180;
    }

    //VARIAVEIS

    private String robName;
    private double irSensor0, irSensor1, irSensor2, irSensor3, compass, compass_goal, x, y, x0,y0;
    //sensores, bussola, bussola para o objetivo, coordenadas, coordenadas iniciais
    // compass_goal para onde ele vai ter de rodar
    private beaconMeasure beacon;
    private int ground;
    private boolean collision,init, end, endS; //initial position?
    private vetor next = new vetor(); //para onde vai a seguir
    private State state;
    private int beaconToFollow;
    private String[][] coords = new String[28][56]; //linhas (Y), colunas (X), mapa completo
    private Set<vetor> coordsAntigas = new HashSet<vetor>(); //linhas, colunas
    private Set<vetor> visitaveis = new HashSet<vetor>();
    
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
        public void printV(){
            System.out.println(x + " "+y);
        }

        public boolean equals(vetor v){
            if(x==v.getX() && y==v.getY())return true;
            return false;
        }
    }
};


