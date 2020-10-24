package com.example.dataclient;

import android.app.Service;
import android.content.Context;
import android.content.Intent;

import android.net.wifi.WifiInfo;
import android.net.wifi.WifiManager;
import android.os.IBinder;
import android.os.PowerManager;
import android.util.Log;

import java.io.BufferedOutputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.DataInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.DataOutputStream;
import java.io.File;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Scanner;
import java.util.ArrayDeque;
import java.util.Iterator;
import java.util.Calendar;
import java.lang.Math; //abs()

import java.net.Socket;//Socket()
import java.net.InetAddress;

import android.content.IntentFilter;
import android.os.Bundle;


public class mmWaveService extends Service {
    private static mmWaveService mmWaveInstance;
    private static Context mContext;

    public static mmWaveService getInstance() {
        if (mmWaveInstance == null) {
            synchronized (mmWaveService.class) {
                if (mmWaveInstance == null) {
                    mmWaveInstance = new mmWaveService();
                }
            }
        }
        return mmWaveInstance;
    }

    private static final String ClassName = mmWaveService.class.getSimpleName();

    private static final int STACK_SIZE = 30;
    private static final int PACKAGE_HEADER_LENGTH = 52;
    private static final int VITAL_SIGNS_CONTENT_LENGTH = 128;
    private static final int NIBBLE = 4;
    private static final int TLV_HEADER_LENGTH = NIBBLE * 2;
    private static final int TLV_TARGET_LIST_LENGTH = NIBBLE * 10;
    private static final int TLV_POINT_CLOUD_LENGTH = NIBBLE * 5;
    private static final int BREATH_FRAME_THRE = 20;
    private static final int FALL_DETECTION_THRE = 9;
    private static final int IN_AREA_THRE = 10; //threshold for count_ManIn/count_NotManIn
    private static final float RCSSTDRAW_AVG_THRE = 700.0f;
    private static final float FIRST_RCSSTDRAW_THRE = 400.0f;

    private static String macAddress = "00:00:00:00:00:00";
    private static String empty = "empty";

    private List<Byte> dataList = new ArrayList<>();
    private int headerCount = 0;

    private int TLV_LieEnabled;
    private int TLV_framenum;
    private int numTLVItem;
    private float FwVersion;
    private String sFwVersion;
    private boolean isRecord = true;
    private float TLV_PosX;
    private float TLV_PosZ;

    private int number;
    private int numbertemp = -1;
    private int count =0;
    private byte[] msg;
    private byte[] nowFramePointCloud;

    private ReadPktThread pktThread;
    private ReadLogThread logThread;

    //連線參數
    private Thread socketClientThread;
    private Socket clientSocket;
    private String tmp;
    private boolean isParsing = false;

    long startTime = 0;
    long endTime = 0;
    long startTime_bed = 0;
    long endTime_bed = 0;
    long startTime_warning = 0;
    long endTime_warning = 0;
    long startTime_fall = 0;
    long endTime_fall = 0;
    long warning = 0;
    //int  leavebed_count = 0; 

    private DetectRadarThread detRadarThread;
    public volatile boolean isRadarPending = false;
    public Object lock = new Object();


    final int SIZE_SAMPLE = 512;

    final int ENERGY_BUF_SIZE = 50;
    final int BR_OUT_BUF_SIZE = 10;


    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }

    @Override
    public void onCreate() {

    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {

        Log.d(ClassName, "Start mmWaveService");
        mContext = mmWaveService.this;

        try {
            socketClientThread = new Thread(Client);
            socketClientThread.start();
            Log.d(ClassName, "Start SocketThread!!");
        } catch (Exception e) {
            Log.d(ClassName, "ConnectService:" + e.getMessage());
        }


        if (logThread == null) {
            logThread = new ReadLogThread(mContext);
            logThread.start();
            Log.d(ClassName, "Start logThread!!");
        }

        if (pktThread == null) {
            pktThread = new ReadPktThread();
            pktThread.start();
            Log.d(ClassName, "Start pktThread!!!");
        }


        return START_STICKY;
    }

    private Runnable Client = new Runnable() {
        @Override
        public void run() {
            InetAddress serverIp;
            while (true) {
                try {
                    serverIp = InetAddress.getByName("192.168.0.105");
                    int serverPort = 5050;
                    clientSocket = new Socket(serverIp, serverPort);
                    break;
                } catch (IOException e) {
                    Log.d(ClassName, "SocketService:" + e.getMessage());
                }
            }
            if (clientSocket.isConnected()) {
                Log.d(ClassName, "conencted server!!!");
                while (clientSocket.isConnected()) {
                    if(isParsing) {
                        try {
                            DataOutputStream bw = new DataOutputStream(new BufferedOutputStream(clientSocket.getOutputStream()));
                            if (numbertemp != number) {
                                bw.write(msg);
                                bw.flush();
                                Log.d(ClassName, "send:" + msg.length);
                            }
                            numbertemp = number;
                        } catch (Exception e) {
                            Log.d(ClassName, "Socketsend:" + e.getMessage());
                            continue;
                        }
                    }
                }
            }
        }
    };

    private class DetectRadarThread extends Thread {
        Context ctx;

        public DetectRadarThread(Context ctx) {
            this.ctx = ctx;
        }

        @Override
        public void run() {
            long startTime = 0;
            long diffTime = 0;
            long currTime = 0;
            long pendingStartTime = 0;
            //Need time to send config to trigger radar, wait for 10s to detect radar.
            try {
                Thread.sleep(10000);
                while (true) {
                    synchronized (lock) {
                        if (isRadarPending) {
                            currTime = System.currentTimeMillis();
                            if (startTime == 0) {
                                startTime = currTime;
                                pendingStartTime = startTime;
                            } else {
                                diffTime = currTime - startTime;
                            }
                            if (diffTime > 3000) { // 3s
                                Log.d(ClassName, "call sendHealthReport(), radar is no response over than 3s.");

                                startTime = currTime;
                            }
                            if ((currTime - pendingStartTime) > 10000) { // 10s
                                Log.d(ClassName, "endTime>10000, try to reboot!!!!!!!");
                                PowerManager pm = (PowerManager) ctx.getSystemService(Context.POWER_SERVICE);
                                pm.reboot(null);
                            }
                        } else {
                            startTime = 0;
                            endTime = 0;
                            currTime = 0;
                            pendingStartTime = 0;
                        }
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private class ReadPktThread extends Thread {
        @Override
        public void run() {
            Byte[] delimiter = {(byte) 0x02, (byte) 0x01, (byte) 0x04, (byte) 0x03, (byte) 0x06, (byte) 0x05, (byte) 0x08, (byte) 0x07};
            byte[] byteData;
            int numRead = -1;

            String line;
            String uart_test = "/system/bin/uart_test";

            try {
                Process p = Runtime.getRuntime().exec(uart_test);
                Log.d(ClassName, "execute uart_test(pkt)");


                BufferedReader br = new
                        BufferedReader(new InputStreamReader(p.getInputStream()));

                while (true) {
                    line = br.readLine();
                    if (line == null)
                        continue;

                    byteData = stdOutputToByte(line);
                    numRead = byteData.length;

                    if (numRead == -1) {
                        Log.d(ClassName, " numRead is -1, abnormal !!!!!! ");

                        continue;

                    }

                    byte[] tempData = new byte[numRead];
                    System.arraycopy(byteData, 0, tempData, 0, numRead);


                    for (byte b : tempData) {
                        dataList.add(b);

                        if (delimiter[headerCount] == b) {
                            headerCount++;
                        } else {
                            headerCount = 0;
                        }

                        if (headerCount == delimiter.length) {
                            dataList = dataList.subList(0, dataList.size() - delimiter.length);
//                            Log.d(ClassName, " update dataList "+ count);
                            parseTLV(listByteToByteArray(dataList));
                            isParsing = true;
                            dataList = new ArrayList<Byte>(Arrays.asList(delimiter));
                            headerCount = 0;
                        }
                    }
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }


    public class ReadLogThread extends Thread {

        Context ctx;

        private String mfilename;
        private DataOutputStream dos;
        private InputStream in;
        private InputStreamReader input;
        private Scanner sc;
        private SimpleDateFormat mSimpleDateFormat = new SimpleDateFormat("YYYY-MM-dd HH:mm:ss.SSS");
        private int filecnt = 0;
        private long fileSizeInBytes;
        private long fileSizeInKB;
        private long fileSizeInMB;
        SimpleDateFormat date = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss,SSS");
        Date currentTime = new Date();
        String[] ReadLogThread = new String[STACK_SIZE];
        String[] ReadLogThread_dbgMsg;
        int num = 0;


        public ReadLogThread(Context ctx) {
            this.ctx = ctx;

        }


        protected void finalize() {
            this.sc.close();
        }

        public String convert(int t) {
            String s = Integer.toString(t);
            if (t < 10) {
                s = "0" + s;
            }
            return s;
        }

        public String getTimeString() {
            Calendar cal = Calendar.getInstance();
            int sec = cal.get(Calendar.SECOND);
            int min = cal.get(Calendar.MINUTE);
            int hour = cal.get(Calendar.HOUR_OF_DAY);
            int date = cal.get(Calendar.DATE);
            int month = cal.get(Calendar.MONTH) + 1;
            int year = cal.get(Calendar.YEAR);
            return convert(year) + convert(month) + convert(date) + convert(hour) + convert(min) + convert(sec);
        }

        @Override
        public void run() {

            String line = null;
            String uart_log = "/system/bin/uart_log";
            int reboot_count = 0;
            long startTime = 0;
            long endTime = 0;

            long startLogTime = 0;
            long currLogTime = 0;

            SimpleDateFormat mAllFormat = new SimpleDateFormat("yyyyMMddHHmm");
            long curtime = System.currentTimeMillis();
            String now_time = mAllFormat.format(new Date(curtime));
            String oldmac = macAddress;

            String newmac = oldmac.replaceAll(":", "_");

            try {

                Process p = Runtime.getRuntime().exec(uart_log);
                Log.d(ClassName, "execute uart_log(config/log)");


                BufferedReader br = new
                        BufferedReader(new InputStreamReader(p.getInputStream()));


                long now, last, diff;

                last = System.currentTimeMillis();


                while (true) {

                    Date d = new Date(System.currentTimeMillis());
                    String logtime = mSimpleDateFormat.format(d);

                    synchronized (lock) {
                        isRadarPending = true;
                    }
                    line = br.readLine();
                    synchronized (lock) {
                        isRadarPending = false;
                    }


                    if ((line != null) && (!line.equals(""))) {

                        now = System.currentTimeMillis();
                        diff = now - last;

                        if (startLogTime == 0) {
                            startLogTime = System.currentTimeMillis();
                            currLogTime = startLogTime;
                        } else {
                            currLogTime = System.currentTimeMillis();
                            if ((currLogTime - startLogTime) >= 3000) { //3 seconds
                                Log.d(ClassName, "call sendHealthReport(), radar is working.");
                                startLogTime = currLogTime;
                            }
                        }

                        last = now;

                        startTime = 0;
                        endTime = 0;

                    }
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
        }

    }

    float range = 0;

    public float getArrayAvg(float[] fArray, int size) {
        int i;
        float total = 0;

        for (i = 0; i < size; i++) {
            total += fArray[i];
        }
        return (total / size);
    }


    private void parseTLV(byte[] fileBytes) {
        SimpleDateFormat date = new SimpleDateFormat("yyyy-MM-dd HH:mm:ss,SSS");
        Date currentTime = new Date();
        String[] tragetObjectData = new String[5];
        String[] pointCloudData = new String[10];
        String[] parseTLV = new String[STACK_SIZE];
        String[] parseTLV_dbgMsg;
        int num = 0;
        //parseTLV1 = new String[num];

        final int OFFSET_RANGEPROFILE_BEGIN = 196;
        final Double ALPHA_RCS = 0.2;
        float cm_Breath;
        float PointX, PointY, PointZ;

        double breath_FFT;
        double breath_Peak;

        int numTLV = 0;
        int TLVType[] = {0, 0, 0, 0, 0, 0};
        int TLVLength[] = {0, 0, 0, 0, 0, 0};

        int IDVirtual = 0;
        //int IDReal = 0;
        int FrameLengthReal;

        boolean TLV0A_Exist = false;
        boolean prefall = false; //Add,2020/6/15

        ArrayList<Integer> Target_IDList = new ArrayList<Integer>();


        // Get the real length of the package
        FrameLengthReal = fileBytes.length;

        // Return if the package only have header
        if (FrameLengthReal < PACKAGE_HEADER_LENGTH)
            return;

        int packetLength = ByteArray2Int(Arrays.copyOfRange(fileBytes, NIBBLE * 5, NIBBLE * 6));
        int frameNumber = ByteArray2Int(Arrays.copyOfRange(fileBytes, NIBBLE * 6, NIBBLE * 7));




        if (FrameLengthReal != packetLength)
            return;

//        msg = fileBytes;
        numTLV = (byte) fileBytes[49] << 8 | (byte) fileBytes[48];


        // This code iterates the TLV package
        try {
            for (int i = 0; i < numTLV; i++) {
                TLVType[i] = geTLVType(fileBytes, PACKAGE_HEADER_LENGTH + sumArray(TLVLength));
                // TLV object length, including TLV header
                TLVLength[i] = geTLVLength(fileBytes, PACKAGE_HEADER_LENGTH + sumArray(TLVLength));
                //if (TLVType[i] == 0x09 || TLVType[i] == 0x02)
                //TLVLength[i] += TLV_HEADER_LENGTH;
            }
        } catch (ArrayIndexOutOfBoundsException e) {
            Log.d(ClassName, "Package Exception: " + e);
            return;
        }
        number = frameNumber;
        Log.d(ClassName,"Framenumber: "+number);
///////////////////////////////////Parser////////////////////////////
        for (int i = 0; i < numTLV; i++) {
            int prefixLength = PACKAGE_HEADER_LENGTH + TLV_HEADER_LENGTH + sumArrayRange(TLVLength, 0, i - 1);
            byte[] byteOutput = Arrays.copyOfRange(fileBytes, prefixLength, prefixLength + TLVLength[i]);

            if (TLVType[i] == 0x06 && TLVLength[i] >= 28) {
                int TLVPoints = (TLVLength[i] - TLV_HEADER_LENGTH) / TLV_POINT_CLOUD_LENGTH;
                byte[] bytesPointCloud = Arrays.copyOfRange(byteOutput,
                        0, TLVPoints * TLV_POINT_CLOUD_LENGTH);
                nowFramePointCloud = bytesPointCloud;
                msg = nowFramePointCloud;
            }
//
//            if (TLVType[i] == 0x07 && TLVLength[i] >= 48) {
//                numTLVItem = (TLVLength[i] - TLV_HEADER_LENGTH) / TLV_TARGET_LIST_LENGTH;
//
//                Log.d(ClassName, "ParseTLV:numTLVItem: " + numTLVItem);
//
//                for (int j = 0; j < numTLVItem; j++) {
//                    int TLVTID = ByteArray2Int(Arrays.copyOfRange(byteOutput,
//                            j * TLV_TARGET_LIST_LENGTH, NIBBLE + j * TLV_TARGET_LIST_LENGTH));
//                    float TLV_PosX = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 2 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                    float TLV_PosY = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE * 2 + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 3 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                    float TLV_PosZ = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE * 3 + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 4 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                    float TLV_DimX = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE * 7 + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 8 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                    float TLV_DimY = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE * 8 + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 9 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                    float TLV_DimZ = ByteBuffer.wrap(Arrays.copyOfRange(byteOutput,
//                            NIBBLE * 9 + j * TLV_TARGET_LIST_LENGTH, NIBBLE * 10 + j * TLV_TARGET_LIST_LENGTH))
//                            .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//
//
//                    if (isRecord) {
//                        tragetObjectData[0] = Integer.toString(frameNumber);
//                        tragetObjectData[1] = Float.toString(TLV_PosX);
//                        tragetObjectData[2] = Float.toString(TLV_PosY);
//                        tragetObjectData[3] = Float.toString(TLV_PosZ);
//                        tragetObjectData[4] = Integer.toString(TLVTID);
//
//
//                    }
//                }
//            }
//
//            if (TLVType[i] == 0x08 && TLVLength[i] > 8) {
//                int numTLVTargetIndex = TLVLength[i] - TLV_HEADER_LENGTH;
//                byte[] TLVTargetIndex = Arrays.copyOfRange(byteOutput, 0, TLVLength[i] - TLV_HEADER_LENGTH);
//
//                int numNowFramePointCloud = (nowFramePointCloud.length) / TLV_POINT_CLOUD_LENGTH;
//                Log.d(ClassName, "FrameNumber: " + frameNumber + "PointCloud: " + numNowFramePointCloud);
//
//                String[][] sendmsg = new String[numNowFramePointCloud][4];
//                if (numTLVTargetIndex == numNowFramePointCloud) {
//                    for (int j = 0; j < numTLVTargetIndex; j++) {
//                        int valTLVTargetIndex = TLVTargetIndex[j] & 0xFF;
//                        float Range = ByteBuffer.wrap(Arrays.copyOfRange(nowFramePointCloud,
//                                j * TLV_POINT_CLOUD_LENGTH, NIBBLE + j * TLV_POINT_CLOUD_LENGTH))
//                                .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                        float ThetaXY = ByteBuffer.wrap(Arrays.copyOfRange(nowFramePointCloud,
//                                NIBBLE + j * TLV_POINT_CLOUD_LENGTH, NIBBLE * 2 + j * TLV_POINT_CLOUD_LENGTH))
//                                .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                        float PhiZ = ByteBuffer.wrap(Arrays.copyOfRange(nowFramePointCloud,
//                                NIBBLE * 2 + j * TLV_POINT_CLOUD_LENGTH, NIBBLE * 3 + j * TLV_POINT_CLOUD_LENGTH))
//                                .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                        float Doppler = ByteBuffer.wrap(Arrays.copyOfRange(nowFramePointCloud,
//                                NIBBLE * 3 + j * TLV_POINT_CLOUD_LENGTH, NIBBLE * 4 + j * TLV_POINT_CLOUD_LENGTH))
//                                .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//                        float SNRValue = ByteBuffer.wrap(Arrays.copyOfRange(nowFramePointCloud,
//                                NIBBLE * 4 + j * TLV_POINT_CLOUD_LENGTH, NIBBLE * 5 + j * TLV_POINT_CLOUD_LENGTH))
//                                .order(ByteOrder.LITTLE_ENDIAN).getFloat();
//
//                        PointX = (float) (Range * Math.cos(PhiZ) * Math.sin(ThetaXY));
//                        PointY = (float) (Range * Math.cos(PhiZ) * Math.cos(ThetaXY));
//                        PointZ = (float) (Range * Math.sin(PhiZ));
//
//                        if (isRecord) {
//                            pointCloudData[0] = Integer.toString(frameNumber);
//                            pointCloudData[1] = Float.toString(PointX);
//                            pointCloudData[2] = Float.toString(PointY);
//                            pointCloudData[3] = Float.toString(PointZ);
//                            pointCloudData[4] = Integer.toString(valTLVTargetIndex);
//                            pointCloudData[5] = Float.toString(Range);
//                            pointCloudData[6] = Float.toString(ThetaXY);
//                            pointCloudData[7] = Float.toString(PhiZ);
//                            pointCloudData[8] = Float.toString(Doppler);
//                            pointCloudData[9] = Float.toString(SNRValue);
//                        }
////                        sendmsg[j][0] = pointCloudData[0];
////                        sendmsg[j][1] = pointCloudData[1];
////                        sendmsg[j][2] = pointCloudData[2];
////                        sendmsg[j][3] = pointCloudData[3];
//                    }
////                    msg = sendmsg;
//                }
//            }
//
//
        }
///////////////////////////////////Parser////////////////////////////
    }

    private int sumArray(int[] array) {
        int sum = 0;

        for (int i : array)
            sum += i;
        return sum;
    }

    private double sumArrayDouble(double[] array) {
        double sum = 0.0;

        for (double i : array)
            sum += i;
        return sum;
    }

    private int sumArrayRange(int[] array, int start, int end) {
        int sum = 0;

        for (int i = start; i <= end; i++)
            sum += array[i];
        return sum;
    }

    private int geTLVType(byte[] fileBytes, int TLV_position) {
        // TLV object type
        int TLV_type = Arrays.copyOfRange(fileBytes, TLV_position, TLV_position + NIBBLE)[0];
        return TLV_type;
    }

    private int geTLVLength(byte[] fileBytes, int TLV_position) {
        // TLV object length, including TLV header length
        int TLV_Length = ByteArray2Int(Arrays.copyOfRange(fileBytes,
                TLV_position + NIBBLE, TLV_position + (NIBBLE * 2)));
        return TLV_Length;
    }


    private static byte[] stdOutputToByte(String output) {
        String[] data = output.trim().split(" ");
        int dataLength = data.length;
        byte[] byteData = new byte[dataLength];

        for (int i = 0; i < dataLength; i++) {
            try {
                byteData[i] = (byte) Integer.parseInt(data[i]);
            } catch (NumberFormatException e) {
                Log.d(ClassName, "NumberFormatException " + e);
            }
        }
        return byteData;
    }

    public static byte[] listByteToByteArray(List<Byte> data) {
        byte[] dataByte = new byte[data.size()];
        for (int i = 0; i < data.size(); i++) {
            dataByte[i] = data.get(i);
        }
        return dataByte;
    }

    public static int ByteArray2Int(byte[] b) {
        int MASK = 0xFF;
        int result = 0;
        result = b[0] & MASK;
        result = result + ((b[1] & MASK) << 8);
        result = result + ((b[2] & MASK) << 16);
        result = result + ((b[3] & MASK) << 24);
        return result;
    }

    public static int ByteArray2Int_two(byte[] b) {
        int MASK = 0xFF;
        int result = 0;
        result = b[0] & MASK;
        result = result + ((b[1] & MASK) << 8);
        //result = result + ((b[2] & MASK) << 16);
        //result = result + ((b[3] & MASK) << 24);
        return result;
    }
    public static byte[] addBytes(byte[] data1, byte[] data2) {
        byte[] data3 = new byte[data1.length + data2.length];
        System.arraycopy(data1, 0, data3, 0, data1.length);
        System.arraycopy(data2, 0, data3, data1.length, data2.length);
        return data3;

    }
    @Override
    public void onDestroy() {
        super.onDestroy();
        Log.d(ClassName, "onDestroy()");
    }

}
