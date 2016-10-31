'Simple example of receiving serial data
'written in Visual Basic
'

Imports System
Imports System.IO.Ports
Imports Excel = Microsoft.Office.Interop.Excel
Imports System.Windows.Forms
Imports System.Threading
Imports System.Text

Public Class USBTestingForm

    Const errTimeRecData = 120
    Const cmdSearch As String = "SEARCH60"
    Dim devIDSearchTime = 60

    '-------Check USB Registry-------
    Const WM_DEVICECHANGE = &H219
    Const DBT_DEVICEARRIVAL = &H8000
    Const DBT_DEVICEREMOVECOMPLETE = &H8004
    Const c_strCP2102 As String = "HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB\VID_10C4&PID_EA60\0001\Device Parameters"
    '--------------------------------

    '-------COM PORT related---------
    Dim comPORT As String
    Dim connectedComPort As String
    Dim receivedData As String = ""
    Dim WithEvents mySerialPort As New SerialPort
    Dim comConnected As Boolean = False

    Private totalLength As Int32 = 0
    Private spRecThread As Thread
    Private Delegate Sub spDataHandle(ByVal buffer As Byte())
    Private Delegate Sub comChangehandler(ByVal stat As comStatus)
    Private Delegate Sub txtBoxHandler(ByVal txt As String)
    Private Delegate Sub recDataHandle(ByVal buffer As Byte())
    Dim tempList As New List(Of Byte)
    Dim dataReceving As Boolean = False
    '--------------------------------

    '-------Data---------------------
    Dim devID As String
    Dim tempValue As Double
    Dim humiValue As Double
    Dim pressValue As Integer
    Dim numberOfDevID As Integer
    Dim dtbRecData As New DataTable("ReceivedData")
    Dim dtbSensorStatus As New DataTable("SensorStatus") 'hidden table to store sensor status, such as last time to recevive data
    Dim dtbSensorStatusrow As DataRow
    Dim dtbDevIDList As New DataTable("DevIDList")
    Dim devIDArray() As String
    Dim getSensorDataCounter As Integer
    Dim getSensorDataCurrentDevIDArrayCount As Integer
    '--------------------------------

    Dim meshONSecCounter As Integer = 0
    Dim meshOFFSecCounter As Integer = 0
    'Private WithEvents timerStatusCheck As New System.Windows.Forms.Timer()

    '------System Status-------------

    Dim statusNVMReadFinish As Boolean = False
    Dim statusRecordInProgress As Boolean = False
    Dim statusSearchingDevID As Boolean = False
    Dim statusComportCMDRecInProgress As Boolean = False
    Dim counterSearchDevID As Integer = 0

    Enum comStatus
        comDisconnected
        comConnected
        comConnecting
    End Enum

    Enum sentCmd
        CONREQ
        REQDEVID
        STRREC
        STPREC
        DEVIDREQ
        DEVIDREAD
        SEARCH
        SEARCHSTOP
        GETDATA
        NONE
    End Enum

    Dim comPortStatus As comStatus
    Dim sentCommand As sentCmd


    Private Sub USBTestingForm_Load(sender As Object, e As EventArgs) Handles MyBase.Load
        '--------initalize value--------
        '----Com port----
        comPORT = ""
        addComToListWithCboUpdate()
        '------------------

        '----Data filter combo box----
        cboFilterDevID.Items.Add("ALL")
        cboFilterDevID.SelectedIndex = 0
        '------------------------------

        '----Init data table----------
        dtbRecData.Columns.Add("DevID", Type.GetType("System.String"))
        dtbRecData.Columns.Add("Date", Type.GetType("System.String"))
        dtbRecData.Columns.Add("Time", Type.GetType("System.String"))
        dtbRecData.Columns.Add("Temp", Type.GetType("System.Double"))
        dtbRecData.Columns.Add("Humi", Type.GetType("System.Double"))
        dtbRecData.Columns.Add("Press", Type.GetType("System.Int32"))
        dtbDevIDList.Columns.Add("col0", Type.GetType("System.String"))
        dtbDevIDList.Columns.Add("col1", Type.GetType("System.String"))
        dtbDevIDList.Columns.Add("col2", Type.GetType("System.String"))
        dtbDevIDList.Columns.Add("col3", Type.GetType("System.String"))
        dtbDevIDList.Columns.Add("col4", Type.GetType("System.String"))
        dtbSensorStatus.Columns.Add("DevID", Type.GetType("System.String"))
        dtbSensorStatus.Columns.Add("Status", Type.GetType("System.String"))
        dtbSensorStatus.Columns.Add("tempRecCounter", Type.GetType("System.Int32"))
        dtbSensorStatus.Columns.Add("Temp", Type.GetType("System.Double"))
        dtbSensorStatus.Columns.Add("humiRecCounter", Type.GetType("System.Int32"))
        dtbSensorStatus.Columns.Add("Humi", Type.GetType("System.Double"))
        dtbSensorStatus.Columns.Add("pressRecCounter", Type.GetType("System.Int32"))
        dtbSensorStatus.Columns.Add("Press", Type.GetType("System.Int32"))
        dtbSensorStatus.Columns.Add("timeTemp", Type.GetType("System.DateTime"))
        dtbSensorStatus.Columns.Add("timeHumi", Type.GetType("System.DateTime"))
        dtbSensorStatus.Columns.Add("timePress", Type.GetType("System.DateTime"))
        dtbSensorStatus.Columns.Add("lasttimeUpdateRecTable", Type.GetType("System.DateTime"))



        Dim PrimaryKeyColumns(0) As DataColumn
        PrimaryKeyColumns(0) = dtbSensorStatus.Columns("DevID")
        dtbSensorStatus.PrimaryKey = PrimaryKeyColumns
        '------------------------------

        '----Init DataGridView--------
        dgvRecData.DataSource = dtbRecData
        '-----------------------------

        '----Init Timer--------
        'timerStatusCheck.Interval = 5000
        timerStatusCheck.Start()
        '-----------------------------

        '----Init System status------
        comPortConnectStatusChange(comStatus.comDisconnected)
        '--------------------------------
#If 0 Then 'test string
        Dim recData() As String = {"SD8001DT30.23TH60.75HP101PE", "SD8002DT28.23TH75.75HP98PE", "SD8003DT29.23TH55.75HP80PE"}



        For k As Integer = 1 To 100
            For Each value As String In recData
                If value.StartsWith("S") Then
                    If value.EndsWith("E") Then
                        devID = Convert.ToInt16(value.Substring(value.IndexOf("D") + 1, value.LastIndexOf("D") - value.IndexOf("D") - 1))
                        'Debug.WriteLine(devID)
                        tempValue = Convert.ToDouble(value.Substring(value.IndexOf("T") + 1, value.LastIndexOf("T") - value.IndexOf("T") - 1))
                        'Debug.WriteLine(tempValue)
                        humiValue = Convert.ToDouble(value.Substring(value.IndexOf("H") + 1, value.LastIndexOf("H") - value.IndexOf("H") - 1))
                        'Debug.WriteLine(humiValue)
                        pressValue = Convert.ToInt16(value.Substring(value.IndexOf("P") + 1, value.LastIndexOf("P") - value.IndexOf("P") - 1))
                        'Debug.WriteLine(pressValue)
                        dtbRecData.Rows.Add(devID, DateTime.Today.ToString("yyyy/MM/dd"), DateTime.Now.ToString("HH:mm"), tempValue, humiValue, pressValue)
                        addDevIDToList()
                    End If
                End If
            Next value
        Next k

        dgvRecData.DataSource = dtbRecData

        dgvRecData.Refresh()
#End If
    End Sub '-----eof form load-----
    Protected Overrides Sub WndProc(ByRef m As System.Windows.Forms.Message)

        If m.Msg = WM_DEVICECHANGE Then

            Select Case m.WParam
                Case WM_DEVICECHANGE

                Case DBT_DEVICEARRIVAL

                    If comPortStatus = comStatus.comDisconnected Then
                        addComToListWithCboUpdate()
                    End If

                    If (comPortStatus = comStatus.comConnecting) Or (comPortStatus = comStatus.comConnected) Then
                        addComToListNoCboUpdate()
                    End If


                Case DBT_DEVICEREMOVECOMPLETE

                    If comPortStatus <> comStatus.comDisconnected Then
                        If checkIfComPortInComPortCBOList(comPORT) = False Then
                            resetComPortCBOList()
                            mySerialPort.Close()
                            comPortConnectStatusChange(comStatus.comDisconnected)
                        End If
                    Else
                        addComToListWithCboUpdate()
                    End If
            End Select
        End If

        MyBase.WndProc(m)

    End Sub

    Private Sub comPortConnectStatusChange(ByVal stat As comStatus)

        Select Case stat
            Case comStatus.comDisconnected
                btnConnect.Text = "Connect"
                btnConnect.Enabled = True
                cboCOMPort.Enabled = True
                btnReadNVM.Enabled = False

                btnStartRec.Text = "Stop Rec"
                btnStartRec.Enabled = False

                btnDelDevID.Enabled = False
                btnSearch.Enabled = False
                btnAddDevID.Enabled = False
                lblConnectionStatus.Text = "Disconnected"
                comPortStatus = comStatus.comDisconnected

                dtbSensorStatus.Clear()
                dtbDevIDList.Clear()
                Erase devIDArray
                numberOfDevID = 0
                lblNumOfDevID.Text = 0
                statusNVMReadFinish = False
                statusRecordInProgress = False

                nudOFFInterval.Enabled = True
                nudONInterval.Enabled = True

                timerMeshON.Dispose()
                timerMeshOFF.Dispose()

            Case comStatus.comConnecting
                btnConnect.Text = "Connecting"
                btnConnect.Enabled = False
                cboCOMPort.Enabled = False
                btnReadNVM.Enabled = False
                btnStartRec.Enabled = False
                btnDelDevID.Enabled = False
                btnSearch.Enabled = False
                btnAddDevID.Enabled = False
                lblConnectionStatus.Text = "Connecting"
                comPortStatus = comStatus.comConnecting

            Case comStatus.comConnected
                btnConnect.Text = "Disconnect"
                btnConnect.Enabled = True
                cboCOMPort.Enabled = False
                btnReadNVM.Enabled = True
                btnStartRec.Enabled = False
                btnStartRec.Text = "Start Rec"
                btnDelDevID.Enabled = False
                btnSearch.Enabled = True
                btnAddDevID.Enabled = False
                lblConnectionStatus.Text = "Connected"
                comPortStatus = comStatus.comConnected

        End Select

    End Sub

    Private Sub addComToListWithCboUpdate()

        Dim intIndex As Integer 'item index for combo list
        Dim comPortFound As Boolean = False 'check if there is any comport found

        cboCOMPort.Items.Clear() 'clear all exist comport first
        comPORT = ""

        For Each sp As String In My.Computer.Ports.SerialPortNames

            intIndex = cboCOMPort.Items.Add(sp) 'add comport to the cbo list and return the index
            comPortFound = True

            If String.Equals(sp, My.Computer.Registry.GetValue(c_strCP2102, "PortName", "")) Then
                cboCOMPort.SelectedIndex = intIndex
                comPORT = cboCOMPort.SelectedItem
            End If

        Next

        If String.Equals(comPORT, "") And comPortFound = True Then 'if no cp2102 found, then select the default index 0
            cboCOMPort.SelectedIndex = 0
            comPORT = cboCOMPort.SelectedItem
        End If

        If String.Equals(comPORT, "") And comPortFound = False Then 'if no comPort found
            cboCOMPort.Items.Add("No Comm Port found")
        End If


    End Sub '---eof addComToListwithupdate

    Private Sub addComToListNoCboUpdate()
        For Each sp As String In My.Computer.Ports.SerialPortNames
            If cboCOMPort.Items.Contains(sp) = False Then
                cboCOMPort.Items.Add(sp)
            End If
        Next
    End Sub '---eof addComToListnoupdate

    Private Sub cboCOMPort_SelectedIndexChanged(sender As Object, e As EventArgs) Handles cboCOMPort.SelectedIndexChanged
        If (cboCOMPort.SelectedItem <> "") Then
            comPORT = cboCOMPort.SelectedItem
        End If
    End Sub

    Private Sub cboFilterDevID_SelectedIndexChanged(sender As Object, e As EventArgs) Handles cboFilterDevID.SelectedIndexChanged
        If (CStr(cboFilterDevID.SelectedItem) <> "ALL") Then 'If not equal to ALL, that means DevId Selected,Filter a new dataview
            Dim view1 As New DataView(dtbRecData)
            view1.RowFilter = "DevID='" & CStr(cboFilterDevID.SelectedItem) & "'" 'Filter DevID=SelectedItem
            dgvRecData.DataSource = view1
            dgvRecData.Refresh()
        Else 'Selected ALL, dgv source as orignal datatable
            dgvRecData.DataSource = dtbRecData
            dgvRecData.Refresh()
        End If
    End Sub

    Private Sub sendCommand(ByVal cmd As String)
        Select Case cmd
            Case "CONREQ"
                sentCommand = sentCmd.CONREQ
            Case "DEVIDREAD"
                sentCommand = sentCmd.DEVIDREAD
            Case "DEVIDREQ"
                sentCommand = sentCmd.DEVIDREQ
            Case "STRREC"
                sentCommand = sentCmd.STRREC
            Case "STPREC"
                sentCommand = sentCmd.STPREC
            Case cmdSearch
                sentCommand = sentCmd.SEARCH
            Case "SEARCHSTOP"
                sentCommand = sentCmd.SEARCHSTOP
        End Select
        Try
            mySerialPort.Write(cmd + "|")
            TextBox1.AppendText(cmd + "|")
        Catch e As Exception
            Select Case comPortStatus
                Case comStatus.comConnecting Or comStatus.comDisconnected
                    comPortConnectStatusChange(comStatus.comDisconnected)
                    mySerialPort.Close()
                    timerRecCONNREQ.Dispose()
                Case Else
                    MessageBox.Show(e.Message)
            End Select
        End Try
    End Sub

    Private Sub serialPortInit()
        Try
            If mySerialPort.IsOpen = False Then
                mySerialPort.Close()
                mySerialPort.PortName = comPORT
                mySerialPort.BaudRate = 115200
                mySerialPort.DataBits = 8
                mySerialPort.Parity = Parity.None
                mySerialPort.StopBits = StopBits.One
                mySerialPort.Handshake = Handshake.None
                mySerialPort.Encoding = System.Text.Encoding.Default
                mySerialPort.ReadTimeout = 500
                AddHandler mySerialPort.DataReceived, AddressOf ReceiveSerialData
                mySerialPort.Open()
            End If
        Catch ex As Exception
            MessageBox.Show(ex.Message)
        End Try

    End Sub

    Private Sub btnClear_Click(sender As Object, e As EventArgs) Handles btnClear.Click
        dtbRecData.Clear()
        dgvRecData.Refresh()
    End Sub

    Private Sub btnDebugSend_Click(sender As Object, e As EventArgs) Handles btnDebugSend.Click
        serialPortInit()
        mySerialPort.Write(txtDebugInput.Text)
    End Sub

    Private Sub btnSimSend_Click(sender As Object, e As EventArgs) Handles btnSimSend.Click
        exeReceivedCPData(System.Text.Encoding.ASCII.GetBytes(txtSim.Text))
    End Sub

    Private Sub btnConnect_Click(sender As Object, e As EventArgs) Handles btnConnect.Click
        Select Case comPortStatus
            Case comStatus.comDisconnected
                serialPortInit()
                comPortConnectStatusChange(comStatus.comConnecting)
                sendCommand("CONREQ")
                timerRecCONNREQ.Start()
            Case comStatus.comConnected
                sendCommand("STPREC")
                sendCommand("SEARCHSTOP")
                mySerialPort.Close()
                comPortConnectStatusChange(comStatus.comDisconnected)
                sentCommand = sentCmd.NONE
        End Select
    End Sub

    Private Sub btnStartRec_Click(sender As Object, e As EventArgs) Handles btnStartRec.Click
        If comPortStatus = comStatus.comConnected Then
            If btnStartRec.Text = "Start Rec" Then
                sendCommand("STRREC")
                btnStartRec.Text = "Stop Rec"
                btnSearch.Enabled = False
                btnDelDevID.Enabled = False
                btnExport.Enabled = False
                btnClear.Enabled = False
                nudONInterval.Enabled = False
                btnAddDevID.Enabled = False
                getSensorDataCounter = 0
                getSensorDataCurrentDevIDArrayCount = 0
            ElseIf btnStartRec.Text = "Stop Rec" Then
                sendCommand("STPREC")
                btnStartRec.Text = "Start Rec"
                btnSearch.Enabled = True
                btnDelDevID.Enabled = True
                btnExport.Enabled = True
                btnClear.Enabled = True
                btnAddDevID.Enabled = True
                getSensorDataCounter = 0
                getSensorDataCurrentDevIDArrayCount = 0
            End If
        Else
            MsgBox("Please connect to USB dongle")
        End If
    End Sub

    Private Sub btnReadNVM_Click(sender As Object, e As EventArgs) Handles btnReadNVM.Click
        If comPortStatus = comStatus.comConnected And statusNVMReadFinish = False Then
            sendCommand("DEVIDREQ")
        End If
    End Sub

    Private Sub btnExport_Click(sender As Object, e As EventArgs) Handles btnExport.Click
        'Using save file dialog that allow you to chosse the file name.
        Dim objDlg As New SaveFileDialog
        Dim saveFileDialogfilepath As String

        objDlg.Filter = "Excel File|*.xls"
        objDlg.OverwritePrompt = False
        Try
            If objDlg.ShowDialog = DialogResult.OK Then
                saveFileDialogfilepath = objDlg.FileName
                dtExportToExcel(dtbRecData, saveFileDialogfilepath)
            End If
        Catch ex As Exception
            MessageBox.Show(ex.Message)
        End Try

    End Sub

    Private Sub btnSearch_Click(sender As Object, e As EventArgs) Handles btnSearch.Click

        If statusSearchingDevID = False Then
            Dim result As DialogResult
            result = MessageBox.Show("All DevID will be deleted, confirm?", "Warning", MessageBoxButtons.YesNo, MessageBoxIcon.Asterisk)
            If result = DialogResult.Yes Then
                dtbDevIDList.Clear()
                dtbSensorStatus.Clear()
                Erase devIDArray
                statusNVMReadFinish = False
                lblNumOfDevID.Text = "0"
                numberOfDevID = 0
                statusSearchingDevID = True
                btnSearch.Text = "Stop Find"
                btnReadNVM.Enabled = False
                btnStartRec.Enabled = False
                btnDelDevID.Enabled = False
                btnAddDevID.Enabled = False
                devIDSearchTime = nudSearchSecond.Value
                txtDevIDStat.Text = "Searching " + CStr(devIDSearchTime - counterSearchDevID) + " s remaining"
                timerSearchDevID.Start()
                If nudONInterval.Value < 10 And nudOFFInterval.Value < 10 Then
                    sendCommand("SO" + "0" + CStr(nudONInterval.Value) + "F" + "0" + CStr(nudOFFInterval.Value))
                ElseIf nudONInterval.Value < 10 And nudOFFInterval.Value > 9 Then
                    sendCommand("SO" + "0" + CStr(nudONInterval.Value) + "F" + CStr(nudOFFInterval.Value))
                ElseIf nudONInterval.Value > 9 And nudOFFInterval.Value < 10 Then
                    sendCommand("SO" + CStr(nudONInterval.Value) + "F" + "0" + CStr(nudOFFInterval.Value))
                ElseIf nudONInterval.Value > 9 And nudOFFInterval.Value > 9 Then
                    sendCommand("SO" + CStr(nudONInterval.Value) + "F" + CStr(nudOFFInterval.Value))
                End If
                'sendCommand("ON" + CStr(nudONInterval.Value) + "OFF" + CStr(nudOFFInterval.Value))
                nudONInterval.Enabled = False
                nudOFFInterval.Enabled = False                
            End If
        Else
            btnSearch.Text = "SEARCH"
            statusSearchingDevID = False
            sendCommand("SEARCHSTOP")
            txtDevIDStat.Text = "Search finished"
            counterSearchDevID = 0
            timerSearchDevID.Stop()
            sendCommand("SEARCHSTOP")
            btnReadNVM.Enabled = True
            nudOFFInterval.Enabled = True
            nudONInterval.Enabled = True
            sendCommand("DEVIDREQ")
        End If

    End Sub

    Private Sub ckbTopMost_CheckedChanged(sender As Object, e As EventArgs) Handles ckbTopMost.CheckedChanged
        If ckbTopMost.Checked Then
            Me.TopMost = True
        Else
            Me.TopMost = False
        End If
    End Sub

    Private Sub addDevIDToCBOFilter() 'extract DevID from received data and add to cbo filter
        Dim drArray() As DataRow = dtbRecData.Select()
        If Not (cboFilterDevID.Items.Contains(drArray(drArray.GetUpperBound(0))(0))) Then
            cboFilterDevID.Items.Add(drArray(drArray.GetUpperBound(0))(0))
        End If
    End Sub

    Private Function checkIfComPortInComPortCBOList(ByVal s As String) As Boolean 'check if comport in the comport list
        For Each sp As String In My.Computer.Ports.SerialPortNames
            If String.Equals(sp, s) Then
                Return True 'if exist, return 1
            End If
        Next
        Return False 'if not exist, return 0
    End Function

    Private Sub resetComPortCBOList()
        cboCOMPort.Items.Clear()
        For Each sp As String In My.Computer.Ports.SerialPortNames
            cboCOMPort.Items.Add(sp)
        Next
        cboCOMPort.SelectedIndex = 0
        comPORT = cboCOMPort.SelectedItem
    End Sub

    Private Sub releaseObject(ByVal obj As Object)
        Try
            System.Runtime.InteropServices.Marshal.ReleaseComObject(obj)
            obj = Nothing
        Catch ex As Exception
            obj = Nothing
        Finally
            GC.Collect()
        End Try
    End Sub
#If 0 Then 'another excel export method
    Private Sub btnExcel_Click(sender As Object, e As EventArgs)

        Dim myExcel As Excel.Application = New Microsoft.Office.Interop.Excel.Application()

        If myExcel Is Nothing Then
            MessageBox.Show("Excel is not properly installed!!")
        Else
            MessageBox.Show("Excel is installed!!")
        End If

        Dim excelWorkBook As Excel.Workbook
        Dim excelWorkSheet As Excel.Worksheet
        Dim misValue As Object = System.Reflection.Missing.Value

        excelWorkBook = myExcel.Workbooks.Add(misValue)
        excelWorkSheet = excelWorkBook.Sheets("sheet1")
        excelWorkSheet.Cells(1, 1) = "Sheet 1 content"

        excelWorkBook.SaveAs("c:\csharp-Excel.xls", Excel.XlFileFormat.xlWorkbookNormal, misValue, misValue, misValue, misValue,
         Excel.XlSaveAsAccessMode.xlExclusive, misValue, misValue, misValue, misValue, misValue)
        excelWorkBook.SaveAs()
        excelWorkBook.Close(True, misValue, misValue)
        myExcel.Quit()

        releaseObject(excelWorkSheet)
        releaseObject(excelWorkBook)
        releaseObject(myExcel)

        MessageBox.Show("Excel file created")
    End Sub
#End If

    Private Sub dtExportToExcel(ByVal dataTable As DataTable, ByVal filepath As String)

        Dim myExcel As Excel.Application = New Microsoft.Office.Interop.Excel.Application()

        Dim excelWorkBook As Excel.Workbook
        Dim excelWorkSheet As Excel.Worksheet
        Dim misValue As Object = System.Reflection.Missing.Value
        Dim strFileName As String = filepath

        If System.IO.File.Exists(strFileName) Then
            If (MessageBox.Show("Overwrite?",
                                "Excel file exist",
                                MessageBoxButtons.YesNo,
                                MessageBoxIcon.Question,
                                MessageBoxDefaultButton.Button2) = System.Windows.Forms.DialogResult.Yes) Then
                System.IO.File.Delete(strFileName)
            Else
                Return
            End If
        End If


        excelWorkBook = myExcel.Workbooks.Add(misValue)
        excelWorkSheet = excelWorkBook.ActiveSheet()

        '-----------------------------------         
        Dim dataCol As System.Data.DataColumn
        Dim dataRow As System.Data.DataRow
        Dim colIndex As Integer = 0
        Dim rowIndex As Integer = 0
        '-------------------------------------

        '---------------------------------- export the Columns          
        'If CheckBox1.Checked Then
        For Each dataCol In dataTable.Columns
            colIndex = colIndex + 1
            excelWorkSheet.Cells(1, colIndex) = dataCol.ColumnName
        Next
        'End If
        '--------------------------------export the rows 
        For Each dataRow In dataTable.Rows
            rowIndex = rowIndex + 1
            colIndex = 0
            For Each dataCol In dataTable.Columns
                colIndex = colIndex + 1
                excelWorkSheet.Cells(rowIndex + 1, colIndex) = dataRow(dataCol.ColumnName)
            Next
        Next

        excelWorkSheet.Columns.AutoFit()

        excelWorkBook.SaveAs(strFileName,
                             Excel.XlFileFormat.xlWorkbookNormal,
                             misValue,
                             misValue,
                             misValue,
                             misValue,
                             Excel.XlSaveAsAccessMode.xlExclusive,
                             misValue,
                             misValue,
                             misValue,
                             misValue,
                             misValue)

        excelWorkBook.Close(True, misValue, misValue)

        myExcel.Quit()

        releaseObject(excelWorkSheet)
        releaseObject(excelWorkBook)
        releaseObject(myExcel)

        MessageBox.Show("Excel file created")

    End Sub

    Private Sub timerRecCONNREQ_Tick(sender As Object, e As EventArgs) Handles timerRecCONNREQ.Tick
        comPortConnectStatusChange(comStatus.comDisconnected)
        mySerialPort.Close()
        timerRecCONNREQ.Dispose()
    End Sub

    Private Sub timerSearchDevID_Tick(sender As Object, e As EventArgs) Handles timerSearchDevID.Tick
        If counterSearchDevID < devIDSearchTime Then
            txtDevIDStat.Text = "Searching " + CStr(devIDSearchTime - counterSearchDevID) + " s remaining"
            counterSearchDevID += 1
        Else
            txtDevIDStat.Text = "Search finished"
            statusSearchingDevID = False
            btnSearch.Text = "SEARCH"
            counterSearchDevID = 0
            timerSearchDevID.Stop()
            sendCommand("SEARCHSTOP")
            btnReadNVM.Enabled = True
            btnDelDevID.Enabled = True
            sendCommand("DEVIDREQ")
        End If
    End Sub

    Private Sub TimerEventProcessor(myObject As Object, ByVal myEventArgs As EventArgs) Handles timerStatusCheck.Tick

        timerStatusCheck.Stop()
        Dim drr As DataRow

        If dtbSensorStatus.Rows.Count > 0 Then
            For i As Integer = 0 To dtbSensorStatus.Rows.Count - 1
                drr = dtbSensorStatus.Rows(i)

                If drr("tempRecCounter") > 0 Or drr("humiRecCounter") > 0 Or drr("pressRecCounter") > 0 Then


                    If DateDiff(DateInterval.Second, drr("timeTemp"), DateTime.Now) > errTimeRecData Or
                        DateDiff(DateInterval.Second, drr("timeHumi"), DateTime.Now) > errTimeRecData Or
                        DateDiff(DateInterval.Second, drr("timePress"), DateTime.Now) > errTimeRecData Then

                        If drr("Status") <> "Data missing" Then
                            drr("Status") = "Data missing"
                            For k As Integer = 0 To dgvDevIDList.Rows.Count - 1
                                For j As Integer = 0 To dgvDevIDList.Columns.Count - 1
                                    Dim CellChange As String = dgvDevIDList.Item(j, k).Value.ToString().Trim()
                                    If CellChange.Contains(drr("DevID")) = True Then
                                        dgvDevIDList.Item(j, k).Value = "ER" + CStr(drr("DevID"))
                                        dgvDevIDList.Item(j, k).Style.ForeColor = Color.Red
                                        dgvDevIDList.ClearSelection()
                                    End If
                                Next
                            Next
                        End If
                    End If
                End If
            Next

        End If

        timerStatusCheck.Enabled = True
    End Sub



    Private Sub ReceiveSerialData()

        Try

            If mySerialPort.IsOpen = False Then
            mySerialPort.Open()
        End If

        While mySerialPort.BytesToRead > 0
                trimSPData(mySerialPort.ReadByte)
            End While
        Catch ex As Exception
            MessageBox.Show(ex.Message)
        End Try
    End Sub

    Private Sub trimSPData(ByVal spdata As Byte)
        Select Case spdata
            Case Encoding.ASCII.GetBytes("[")(0)
                tempList.Clear()
                tempList.Add(CType(spdata, Byte))
                statusComportCMDRecInProgress = True

            Case Encoding.ASCII.GetBytes("]")(0)
                tempList.Add(CType(spdata, Byte))
                If statusComportCMDRecInProgress = True Then
                    If tempList(0) = Encoding.ASCII.GetBytes("[")(0) AndAlso tempList(tempList.Count - 1) = Encoding.ASCII.GetBytes("]")(0) Then
                        tempList.RemoveAt(0)
                        tempList.RemoveAt(tempList.Count - 1)
                        Dim d As New recDataHandle(AddressOf exeReceivedCPData)
                        Me.Invoke(d, New Object() {tempList.ToArray()})
                        Dim txtBox As New txtBoxHandler(AddressOf outTotxtBox)
                    End If
                    statusComportCMDRecInProgress = False
                    tempList.Clear()
                End If
            Case Else
                If statusComportCMDRecInProgress = True Then
                    tempList.Add(CType(spdata, Byte))
                Else
                    Return
                End If
        End Select
    End Sub

    Private Sub outTotxtBox(ByVal a As String)
        TextBox1.AppendText(a)
    End Sub

    Private Sub exeReceivedCPData(ByVal buffer As Byte())

        Dim cmd As String = System.Text.Encoding.ASCII.GetString(buffer)
        TextBox1.AppendText(cmd)

        If cmd.StartsWith("CONREQOK") Then
            timerRecCONNREQ.Dispose()
            comPortStatus = comStatus.comConnected
            Dim comchange As New comChangehandler(AddressOf comPortConnectStatusChange)
            Me.Invoke(comchange, comStatus.comConnected)
        End If

        If btnStartRec.Text = "Stop Rec" Then

            If cmd.StartsWith("$RT") And dtbSensorStatus.Rows.Count > 0 Then
                devID = cmd.Substring(cmd.IndexOf("D") + 1, cmd.LastIndexOf("D") - cmd.IndexOf("D") - 1)
                tempValue = Convert.ToDouble(cmd.Substring(cmd.IndexOf("DT") + 2, cmd.LastIndexOf("T") - (cmd.IndexOf("DT") + 2)))
                Dim drr As DataRow
                drr = dtbSensorStatus.Rows.Find(devID)
                If Not (dtbSensorStatus.Rows.Find(devID) Is Nothing) Then
                    drr("Temp") = tempValue
                    drr("timeTemp") = DateTime.Now
                    drr("tempRecCounter") = drr("tempRecCounter") + 1
                    If drr("tempRecCounter") > 0 And drr("humiRecCounter") > 0 And drr("pressRecCounter") > 0 And DateDiff(DateInterval.Second, drr("lasttimeUpdateRecTable"), DateTime.Now) > 120 Then
                        drr("tempRecCounter") = 0
                        drr("humiRecCounter") = 0
                        drr("pressRecCounter") = 0
                        updateData(devID)
                        drr("lasttimeUpdateRecTable") = DateTime.Now
                    End If
                End If
            End If


            If cmd.StartsWith("$RH") And dtbSensorStatus.Rows.Count > 0 Then
                devID = cmd.Substring(cmd.IndexOf("D") + 1, cmd.LastIndexOf("D") - cmd.IndexOf("D") - 1)
                humiValue = Convert.ToDouble(cmd.Substring(cmd.IndexOf("DH") + 2, cmd.LastIndexOf("H") - (cmd.IndexOf("DH") + 2)))
                Dim drr As DataRow
                drr = dtbSensorStatus.Rows.Find(devID)
                If Not (dtbSensorStatus.Rows.Find(devID) Is Nothing) Then
                    drr("Humi") = humiValue
                    drr("timeHumi") = DateTime.Now
                    drr("humiRecCounter") = drr("humiRecCounter") + 1
                    Console.WriteLine(drr("lasttimeUpdateRecTable"))
                    Console.WriteLine(DateTime.Now)
                    Console.WriteLine(DateDiff(DateInterval.Second, drr("lasttimeUpdateRecTable"), DateTime.Now))
                    Console.WriteLine(drr("humiRecCounter"))

                    If drr("tempRecCounter") > 0 And drr("humiRecCounter") > 0 And drr("pressRecCounter") > 0 And DateDiff(DateInterval.Second, drr("lasttimeUpdateRecTable"), DateTime.Now) > 120 Then
                        drr("tempRecCounter") = 0
                        drr("humiRecCounter") = 0
                        drr("pressRecCounter") = 0
                        updateData(devID)
                        drr("lasttimeUpdateRecTable") = DateTime.Now
                    End If
                End If
            End If

            If cmd.StartsWith("$RP") And dtbSensorStatus.Rows.Count > 0 Then
                devID = cmd.Substring(cmd.IndexOf("D") + 1, cmd.LastIndexOf("D") - cmd.IndexOf("D") - 1)
                pressValue = Convert.ToDouble(cmd.Substring(cmd.IndexOf("DP") + 2, cmd.LastIndexOf("P") - (cmd.IndexOf("DP") + 2)))
                Dim drr As DataRow
                drr = dtbSensorStatus.Rows.Find(devID)
                If Not (dtbSensorStatus.Rows.Find(devID) Is Nothing) Then
                    drr("Press") = pressValue
                    drr("timePress") = DateTime.Now
                    drr("pressRecCounter") = drr("pressRecCounter") + 1
                    If drr("tempRecCounter") > 0 And drr("humiRecCounter") > 0 And drr("pressRecCounter") > 0 And DateDiff(DateInterval.Second, drr("lasttimeUpdateRecTable"), DateTime.Now) > 120 Then
                        drr("tempRecCounter") = 0
                        drr("humiRecCounter") = 0
                        drr("pressRecCounter") = 0
                        updateData(devID)
                        drr("lasttimeUpdateRecTable") = DateTime.Now
                    End If
                End If
            End If

        End If


        If cmd.StartsWith("DEVIDREQ") Then
            Try
                numberOfDevID = Convert.ToInt16(cmd.Substring(cmd.IndexOf("Q") + 1, cmd.Length - cmd.IndexOf("Q") - 1))
            Catch
                Return
            End Try

            If numberOfDevID > 0 Then
                sendCommand("DEVIDREAD")
            End If
        End If

        If cmd.StartsWith("SEARCHSTARTSTOP") Then
            sendCommand("DEVIDREQ")
        End If

        If cmd.StartsWith("IDS") Then

            Dim devidData As String = cmd
            Dim stringSeparators() As String = {"/"}
            devidData = devidData.Remove(devidData.IndexOf("IDS"), 3)
            'devidData = devidData.Remove(devidData.IndexOf(""), 2)
            Console.Write(devidData)
            devIDArray = devidData.Split(stringSeparators, StringSplitOptions.None)
            lblNumOfDevID.Text = devIDArray.Count

            dtbSensorStatus.Clear()
            dtbDevIDList.Clear()


            For j As Integer = 0 To devIDArray.Count - 1
                dtbSensorStatusrow = dtbSensorStatus.NewRow()
                dtbSensorStatusrow("DevID") = devIDArray(j)
                dtbSensorStatusrow("Status") = "ID Received"
                dtbSensorStatusrow("Temp") = 0
                dtbSensorStatusrow("Humi") = 0
                dtbSensorStatusrow("Press") = 0
                dtbSensorStatusrow("tempRecCounter") = 0
                dtbSensorStatusrow("humiRecCounter") = 0
                dtbSensorStatusrow("pressRecCounter") = 0
                dtbSensorStatusrow("timeTemp") = DateTime.Now
                dtbSensorStatusrow("timeHumi") = DateTime.Now
                dtbSensorStatusrow("timePress") = DateTime.Now
                dtbSensorStatusrow("lasttimeUpdateRecTable") = "2016/8/29"
                dtbSensorStatus.Rows.Add(dtbSensorStatusrow)
            Next

            If devIDArray.Count > 0 Then
                statusNVMReadFinish = True
                btnReadNVM.Enabled = False
                btnSearch.Enabled = True
                btnDelDevID.Enabled = True
                btnStartRec.Enabled = True
            End If

            updateDevIDDataGridView()

        End If

        If cmd.StartsWith("$READY") And dtbSensorStatus.Rows.Count > 0 Then 'need to chech if sensors are read from nvm
            meshONSecCounter = 0
            timerMeshON.Start()
            timerMeshOFF.Dispose()

        End If

        If cmd.StartsWith("$END") And dtbSensorStatus.Rows.Count > 0 Then 'need to chech if sensors are read from nvm
            meshOFFSecCounter = 0
            timerMeshOFF.Start()
            timerMeshON.Dispose()
            meshONSecCounter = 0
            getSensorDataCounter = 0
            getSensorDataCurrentDevIDArrayCount = 0
        End If

        If cmd.StartsWith("REMOVEALLOK") Then
            dtbDevIDList.Clear()
            dtbSensorStatus.Clear()
            Erase devIDArray
            statusNVMReadFinish = False
            lblNumOfDevID.Text = "0"
            numberOfDevID = 0
            btnAddDevID.Enabled = True
            btnStartRec.Enabled = False
            btnStartRec.Text = "Start Rec"
            btnSearch.Enabled = True
            btnDelDevID.Enabled = True
            btnExport.Enabled = True
            btnClear.Enabled = True
            btnAddDevID.Enabled = True
            getSensorDataCounter = 0
            getSensorDataCurrentDevIDArrayCount = 0
        End If


    End Sub

    Private Sub updateDevIDDataGridView()
        Dim i As Integer = 0
        dtbDevIDList.Clear()
        While i < devIDArray.Count
            If devIDArray.Count - i > 4 Then
                dtbDevIDList.Rows.Add(devIDArray(i), devIDArray(i + 1), devIDArray(i + 2), devIDArray(i + 3), devIDArray(i + 4))
                i = i + 5
            ElseIf devIDArray.Count - i > 3 Then
                dtbDevIDList.Rows.Add(devIDArray(i), devIDArray(i + 1), devIDArray(i + 2), devIDArray(i + 3))
                i = i + 4
            ElseIf devIDArray.Count - i > 2 Then
                dtbDevIDList.Rows.Add(devIDArray(i), devIDArray(i + 1), devIDArray(i + 2))
                i = i + 3
            ElseIf devIDArray.Count - i > 1 Then
                dtbDevIDList.Rows.Add(devIDArray(i), devIDArray(i + 1))
                i = i + 2
            ElseIf devIDArray.Count - i > 0 Then
                dtbDevIDList.Rows.Add(devIDArray(i))
                i = i + 1
            End If
        End While

        dgvDevIDList.DataSource = dtbDevIDList
        dgvDevIDList.Refresh()
        dgvDevIDList.Columns(0).DefaultCellStyle.BackColor = Color.Red
    End Sub

    Private Sub updateData(devID As String)
        If btnStartRec.Text = "Stop Rec" Then
            Dim drr As DataRow
            drr = dtbSensorStatus.Rows.Find(devID)
            dtbRecData.Rows.Add(devID, DateTime.Today.ToString("yyyy/MM/dd"), DateTime.Now.ToString("HH:mm"), drr("Temp"), drr("Humi"), drr("Press"))
            addDevIDToCBOFilter()
            dgvRecData.DataSource = dtbRecData
            dgvRecData.Refresh()
            Console.Write("OK")
            drr("tempRecCounter") = 0
            drr("humiRecCounter") = 0
            drr("pressRecCounter") = 0
            drr("Temp") = 0
            drr("Humi") = 0
            drr("Press") = 0
            drr("Status") = "OK"

            timerMeshON.Stop()
            meshONSecCounter = 0
            If getSensorDataCurrentDevIDArrayCount < devIDArray.Count - 1 Then
                getSensorDataCurrentDevIDArrayCount += 1
            Else
                getSensorDataCurrentDevIDArrayCount = 0
            End If
            timerMeshON.Start()

            sendCommand("update tb ok !!!")

        End If
    End Sub

    Private Sub dgvDevIDList_CellMouseClick(sender As Object, e As DataGridViewCellMouseEventArgs) Handles dgvDevIDList.CellMouseClick
        'txtDevIDStat.Text = devIDDict.Item(dgvDevIDList.CurrentCell.Value)        
        Try
            Dim cellContent As String = dgvDevIDList.CurrentCell.Value
            If cellContent.Contains("ER") = True Then
                cellContent = cellContent.Substring(2)
                txtDevIDStat.Text = cellContent + ":" + dtbSensorStatus.Rows.Find(cellContent)("Status")
            Else
                txtDevIDStat.Text = dgvDevIDList.CurrentCell.Value + ":" + dtbSensorStatus.Rows.Find(dgvDevIDList.CurrentCell.Value)("Status")
            End If
            dgvDevIDList.ClearSelection()
        Catch
        End Try
    End Sub

    Private Sub dgvDevIDList_CellMouseDoubleClick(sender As Object, e As DataGridViewCellMouseEventArgs) Handles dgvDevIDList.CellMouseDoubleClick
        Try
            Dim cellContent As String = dgvDevIDList.CurrentCell.Value
            If cellContent.Contains("ER") = True Then
                dgvDevIDList.CurrentCell.Value = cellContent.Substring(2)
                dgvDevIDList.CurrentCell.Style.ForeColor = Color.Black
                Dim drr As DataRow
                drr = dtbSensorStatus.Rows.Find(dgvDevIDList.CurrentCell.Value)
                drr("Status") = "ID Received"
                drr("tempRecCounter") = 0
                drr("humiRecCounter") = 0
                drr("pressRecCounter") = 0
                drr("Temp") = 0
                drr("Humi") = 0
                drr("Press") = 0
                drr("timeTemp") = DateTime.Now
                drr("timeHumi") = DateTime.Now
                drr("timePress") = DateTime.Now
                txtDevIDStat.Text = dgvDevIDList.CurrentCell.Value + ":" + dtbSensorStatus.Rows.Find(dgvDevIDList.CurrentCell.Value)("Status")
            Else
                txtDevIDStat.Text = dgvDevIDList.CurrentCell.Value + ":" + dtbSensorStatus.Rows.Find(dgvDevIDList.CurrentCell.Value)("Status")
            End If
            dgvDevIDList.ClearSelection()
        Catch
        End Try
    End Sub

    Private Sub timerMeshON_Tick(sender As Object, e As EventArgs) Handles timerMeshON.Tick
        meshONSecCounter += 1
        If btnStartRec.Text = "Stop Rec" Then
            If meshONSecCounter = 5 Or meshONSecCounter = 1 Then
                If devIDArray.Count > 0 Then
                    getSensorDataCounter += 1
                    If getSensorDataCounter < 3 Then
                        sendCommand("G" + devIDArray(getSensorDataCurrentDevIDArrayCount))
                    Else
                        getSensorDataCounter = 0
                        meshONSecCounter = 0
                        If getSensorDataCurrentDevIDArrayCount < devIDArray.Count - 1 Then
                            getSensorDataCurrentDevIDArrayCount += 1
                        Else
                            getSensorDataCurrentDevIDArrayCount = 0
                        End If
                    End If
                End If
            End If
        End If
    End Sub

    Private Sub timerMeshOFF_Tick(sender As Object, e As EventArgs) Handles timerMeshOFF.Tick
        meshOFFSecCounter += 1
    End Sub

    Private Sub btnAddDevID_Click(sender As Object, e As EventArgs) Handles btnAddDevID.Click
        Dim syncTime As Integer = 0
        Dim duration As Integer = nudONInterval.Value * 60
        Dim devID As String = txtDevIDStat.Text

        If timerMeshON.Enabled = False And timerMeshON.Enabled = False Then

        ElseIf meshONSecCounter = 0 Then
            syncTime = duration - meshOFFSecCounter - 2
            sendCommand("D" + devID + "OFF" + CStr(syncTime))
        ElseIf meshOFFSecCounter = 0 Then
            syncTime = duration - meshONSecCounter - 2
            sendCommand("D" + devID + "ON" + CStr(syncTime))
        End If

    End Sub

    Private Sub btnDelDevID_Click(sender As Object, e As EventArgs) Handles btnDelDevID.Click
        sendCommand("REMOVEALL")
    End Sub

End Class