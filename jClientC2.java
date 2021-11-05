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
//Pergunta professor: rodar e como meter angulos diffs nos sensores e coords gps sempre as mesmas no x0 e y0?
//Acabar os cruzamentos 1 e 0
//Acabar funcao rodar
//Add funcao para guardar coordenadas das paredes 
//funcao para se paredes sao verticais ou  horizontais
//funcao para verificar se ja viu o mapa todo
//funcao para ver se celula é inacessivel
//funcao para calcular coordenadas dos limites
//funcao para o beco, voltar para tras para a celula mais proxima
//funcao para dar print no mapa

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

    enum State {RUN, WAIT, RETURN}

    public static void main(String[] args) {

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
        double[] angles = {-90,90};
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

            state = State.RUN;
    }

    /** 
     * reads a new message, decides what to do and sends action to simulator
     */
    public void mainLoop () {
        cif.ReadSensors();
        x0=cif.GetX();
        y0=cif.GetY();
        while(true) {
                cif.ReadSensors();
                decide();
        }
    }


     /*! In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
     *  to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
     */
    //Array guarda informacao e imprimi mapa no fim PrintMapa
    //Aprender a usar GPS e 
    //bussola double = angulo
    //cruzamento = 1 parede 3 espacos (vem de x1, vai a x2, volta, vai a x3, volta, volta para x1)
    //beco = 3 paredes 1 espaco
    //Canto inferior esquerdo do mapa é o (0,0)
    public void wander(boolean followBeacon) {
        if(x!=next.getX() || y!=next.getY()){ //andar em frente
            double dist=Math.max(Math.abs(x-next.getX()), Math.abs(y-next.getY()));
            if(dist<0.15)cif.DriveMotors(dist, dist);
            cif.DriveMotors(0.15, 0.15);
            System.out.println("A caminho do next!");
        }else{
            CoordAntigas.add(atual); //x,y atuais adicionados à ultima posicao array
            if(deadlock()==2){
                if(compass==0){
                    if(ParedeEsquerda() && ParedeDireita()) { // vai para a frente dele
                        next.setX(x+2);
                        next.setY(y);
                    }else if(ParedeEsquerda() && ParedeFrente()) { // vai para a direita dele
                        next.setX(x);
                        next.setY(y-2);
                        rodar_90();
                    }else if(ParedeFrente() && ParedeDireita()) { // vai para esquerda dele
                        next.setX(x);
                        next.setY(y+2);
                        rodar90();
                    }
                }else if(compass==90){
                    if(ParedeEsquerda() && ParedeDireita()) { // vai para a frente dele
                        next.setX(x);
                        next.setY(y+2);
                    }else if(ParedeEsquerda() && ParedeFrente()) { // vai para a direita dele
                        next.setX(x+2);
                        next.setY(y);
                        rodar_90();
                    }else if(ParedeFrente() && ParedeDireita()) { // vai para esquerda dele
                        next.setX(x-2);
                        next.setY(y);
                        rodar90();
                    }
                }else if(compass==-90){
                    if(ParedeEsquerda() && ParedeDireita()) { // vai para a frente dele
                        next.setX(x);
                        next.setY(y-2);
                    }else if(ParedeEsquerda() && ParedeFrente()) { // vai para a direita dele
                        next.setX(x-2);
                        next.setY(y);
                        rodar_90();
                    }else if(ParedeFrente() && ParedeDireita()) { // vai para esquerda dele
                        next.setX(x+2);
                        next.setY(y);
                        rodar90();
                    }
                }else if (compass == -180 || compass ==180){
                    if(ParedeEsquerda() && ParedeDireita()) { // vai para a frente dele
                        next.setX(x-2);
                        next.setY(y);
                    }else if(ParedeEsquerda() && ParedeFrente()) { // vai para a direita dele
                        next.setX(x);
                        next.setY(y+2);
                        rodar_90();
                    }else if(ParedeFrente() && ParedeDireita()) { // vai para esquerda dele
                        next.setX(x);
                        next.setY(y-2);
                        rodar90();
                    }
                }else{
                    alinhar();
                }
            System.out.println("Coordenadas calculadas do next - x= " + next.getX()+" y= "+next.getY());


            }else if (deadlock()==3){  
                
            }else if (deadlock()==1){ 
                if (ParedeFrente()){
                    if(CoordAntigas.contains(coordEsq())) {
                        next.setXY(coordDir());
                        CoordCruz.remove(atual);
                    }
                    else if(CoordAntigas.contains(coordDir())){
                        next.setXY(coordEsq());
                        CoordCruz.remove(atual);
                    }
                    else if (!CoordAntigas.contains(coordDir()) && !CoordAntigas.contains(coordEsq())){
                        next.setXY(coordDir());
                        CoordCruz.add(atual);
                    }else { //visitar o cruzamento mais proximo e ver se ja  viu tudo
                        CoordCruz.remove(atual);
                    } 
                }else if(ParedeEsquerda()){
                    if(CoordAntigas.contains(coordFrente())) {
                        next.setXY(coordDir());
                        CoordCruz.remove(atual);
                    }else if(CoordAntigas.contains(coordDir())){ 
                        next.setXY(coordFrente());
                        CoordCruz.remove(atual);    
                    }
                    else if (!CoordAntigas.contains(coordDir()) && !CoordAntigas.contains(coordFrente())){
                        next.setXY(coordDir());
                        CoordCruz.add(atual);
                    }else{//visitar o cruzamento mais proximo e ver se ja  viu tudo
                        CoordCruz.remove(atual); 
                    }
                }
                else if(ParedeDireita()){
                     if(CoordAntigas.contains(coordEsq())) {
                        next.setXY(coordFrente());
                        CoordCruz.remove(atual);
                    }
                    else if(CoordAntigas.contains(coordFrente())){
                        next.setXY(coordEsq());
                        CoordCruz.remove(atual);
                    }
                    else if (!CoordAntigas.contains(coordFrente()) && !CoordAntigas.contains(coordEsq())){
                        next.setXY(coordEsq());
                        CoordCruz.add(atual);
                    }else { //visitar o cruzamento mais proximo e ver se ja  viu tudo
                        CoordCruz.remove(atual);
                    } 
                }
            }
        }
    }

    //----------------------------------------Funcoes Auxiliares------------------------
    //----------------------------------------Funcoes Auxiliares------------------------
    //----------------------------------------Funcoes Auxiliares------------------------
    //----------------------------------------Funcoes Auxiliares------------------------
    //----------------------------------------Funcoes Auxiliares------------------------


    public void rodar90(){ //rodar 90 graus
        // for(int i=0; i< ; i ++){
        //     cif.DriveMotors(-0.15, 0.15);
        //     cif.ReadSensors();            
        // }
        // cif.ReadSensors();
        // cif.DriveMotors();
    }
    public void rodar_90(){ //rodar -90 graus
        // for(int i=0; i< ; i ++){
        //     cif.DriveMotors(0.15,-0.15);
        //     cif.ReadSensors();            
        // }
        // cif.ReadSensors();
        // cif.DriveMotors();
    }
    public void rodar180(){ //rodar 180 graus
        // for(int i=0; i< ; i ++){
        //     cif.DriveMotors(-0.15, 0.15);
        //     cif.ReadSensors();            
        // }
        // cif.ReadSensors();
        // cif.DriveMotors();
    }

    //serie de funcoes para dar coordenadas dos vizinhos
    public vetor coordEsq(){
        vetor v= new vetor(0,0);
        if(compass==0) v.setXY(x,y+2);
        else if(compass==90) v.setXY(x-2,y);
        else if(compass==-90) v.setXY(x+2,y);
        else v.setXY(x,y-2);
        return v;
    }
    public vetor coordDir(){
        vetor v= new vetor(0,0);
        if(compass==0) v.setXY(x,y-2);
        else if(compass==90) v.setXY(x+2,y);
        else if(compass==-90) v.setXY(x-2,y);
        else v.setXY(x,y+2);
        return v;
    }
    public vetor coordFrente(){
        vetor v= new vetor(0,0);
        if(compass==0) v.setXY(x+2,y);
        else if(compass==90) v.setXY(x,y+2);
        else if(compass==-90) v.setXY(x,y-2);
        else v.setXY(x-2,y);
        return v;
    }
    public vetor coordTras(){
        vetor v= new vetor(0,0);
        if(compass==0)  v.setXY(x-2,y);
        else if(compass==90)  v.setXY(x,y-2);
        else if(compass==-90)  v.setXY(x,y+2);
        else  v.setXY(x+2,y);
        return v;
    }

    //ver se existe parede nas 4 direcoes
    public boolean ParedeFrente(){
        if(irSensor0>=2.5) return true;
        return false;
    }
    public boolean ParedeTras(){
        if(irSensor3>=2.5) return true;
        return false;
    }
    public boolean ParedeDireita(){
        if(irSensor2>=2.5) return true;
        return false;
    }
    public boolean ParedeEsquerda(){
        if(irSensor1>=2.5) return true;
        return false;
    }

    public int deadlock(){ //return 2 "normal", 3 "beco", 1 ou 0 "cruzamento";
        int c=0;
        if(irSensor0>=2.5) c++;
        if(irSensor1>=2.5) c++;
        if(irSensor2>=2.5) c++;
        if(irSensor3>=2.5) c++;
        return c;
    }

    public void alinhar(){ //vai alinhar o robo para o angulo mais proximo
        if(compass<0){
            if(compass>=-45){
                 //rodar para o 0
            }else if(compass>=135){
                //rodar para -90
            }else{
              //rodar para -180  
            }
        }else{
            if (compass<=45){
                //rodar para o 0
            }else if(compass<=135){
                //para o 90
            }else{
                //rodar para os 180
            }
        }
    }
    /**
     * basic reactive decision algorithm, decides action based on current sensor values
     */
    public void decide() {
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

            //System.out.println(robName + " state " + state);

            switch(state) {
                 case RUN:    /* Go */
                     if( cif.GetVisitingLed() ) state = State.WAIT;
                     if( ground == 0 ) {         /* Visit Target */
                         cif.SetVisitingLed(true);
                         System.out.println(robName + " visited target at " + cif.GetTime() + "\n");
                     }

                     else {
                         wander(false);
                     }
                     break;
                 case WAIT: /* set returning led and check that it is on */
                     cif.SetReturningLed(true);
                     if(cif.GetVisitingLed()) cif.SetVisitingLed(false);
                     if(cif.GetReturningLed()) state = State.RETURN;

                     cif.DriveMotors(0.0,0.0);
                     break;
                 case RETURN: /* Return to home area */
                     cif.SetVisitingLed(false);
                     cif.SetReturningLed(false);
                     wander(false);
                     break;

            }
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

    private String robName;
    private double irSensor0, irSensor1, irSensor2, irSensor3, compass, x,y, x0,y0;
    private LinkedList<vetor> CoordAntigas = new LinkedList<vetor>(); //onde ja esteve
    private LinkedList<vetor> CoordCruz = new LinkedList<vetor>(); // coordenadas do cruzamentos
    private beaconMeasure beacon;
    private int ground;
    private boolean collision;
    private vetor next = new vetor(); //para onde vai a seguir
    private vetor atual = new vetor(x,y); //vetor coordenadas atuais
    private State state;

    private int beaconToFollow;

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
        public void setXY(double x, double y){
            this.x=x;
            this.y=y;
        }
    } 
};

 
