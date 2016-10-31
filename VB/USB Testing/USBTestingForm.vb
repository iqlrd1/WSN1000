'Simple example of receiving serial data
'written in Visual Basic
'

Imports System
Imports System.IO.Ports
Imports Excel = Microsoft.Office.Interop.Excel
Imports System.Collections.ObjectModel

'Public Class dataFormat
'Public Property DevID As String
'Public Property RecTime As Integer
'Public Property Temperature As Integer
'Public Property Humidity As Integer
'Public Property Pressure As Integer
'End Class

'Public Class devData
'Inherits ObservableCollection(Of dataFormat)
'End Class

Public Class USBTestingForm

    Const WM_DEVICECHANGE = &H219
    Const DBT_DEVICEARRIVAL = &H8000
    Const DBT_DEVICEREMOVECOMPLETE = &H8004
    Const c_strCP2102 As String = "HKEY_LOCAL_MACHINE\SYSTEM\CurrentControlSet\Enum\USB\VID_10C4&PID_EA60\0001\Device Parameters"

    Dim comPORT As String
    Dim receivedData As String = ""
    Dim WithEvents mySerialPort As New SerialPort
    Dim comDetected As Boolean = False
    Dim comConnected As Boolean = False

   ' Private WithEvents source As devData

    Protected Overrides Sub WndProc(ByRef m As System.Windows.Forms.Message)

        If m.Msg = WM_DEVICECHANGE Then

            Select Case m.WParam
                Case WM_DEVICECHANGE
                Case DBT_DEVICEARRIVAL
                    If Not (comDetected) Then
                        addComToList()
                    End If
                Case DBT_DEVICEREMOVECOMPLETE
                    If comDetected Then
                        If Not (checkComPort(comPORT)) Then
                            resetCboList()
                            mySerialPort.Close()
                            btnConnect.Text = "Connect"
                            comDetected = False
                            comConnected = False
                        End If
                    End If

            End Select
        End If

        MyBase.WndProc(m)

    End Sub


    Private Sub USBTestingForm_Load(sender As Object, e As EventArgs) Handles MyBase.Load

        comPORT = ""
        comDetected = False
        addComToList()

        'Dim D8001 As New dataFormat With
        '   {.DevID = "8001", .RecTime = 13, .Temperature = 14, .Humidity = 15, .Pressure = 16}
        'Dim D8002 As New dataFormat With
        '   {.DevID = "8002", .RecTime = 17, .Temperature = 18, .Humidity = 19, .Pressure = 20}

        '        source = New devData
        '       source.Add(D8001)
        '      source.Add(D8002)
        'Me.Grid1.DataContext = source

        'If you plan to data-bind only the DataGrid:
        'Me.DataGrid1.ItemsSource = source



    End Sub



    Private Sub cboCOMPort_SelectedIndexChanged(sender As Object, e As EventArgs) Handles cboCOMPort.SelectedIndexChanged
        If (cboCOMPort.SelectedItem <> "") Then
            comPORT = cboCOMPort.SelectedItem
        End If
    End Sub

    Private Sub btnConnect_Click(sender As Object, e As EventArgs) Handles btnConnect.Click

        If (btnConnect.Text = "Connect") Then
            If (comPORT <> "") Then
                mySerialPort.PortName = comPORT
                mySerialPort.Close()
                mySerialPort.BaudRate = 2400
                mySerialPort.DataBits = 8
                mySerialPort.Parity = Parity.None
                mySerialPort.StopBits = StopBits.One
                mySerialPort.Handshake = Handshake.None
                mySerialPort.Encoding = System.Text.Encoding.Default
                mySerialPort.ReadTimeout = 10000
                mySerialPort.Open()
                comConnected = True
                btnConnect.Text = "Disconnect"
            Else
                MsgBox("Select a COM port first")
            End If
        Else
            mySerialPort.Close()
            comConnected = False
            btnConnect.Text = "Connect"
        End If
    End Sub

    Private Sub btnClear_Click(sender As Object, e As EventArgs) Handles btnClear.Click
        txtOutput.Text = ""
    End Sub

    Private Sub btnSend_Click(sender As Object, e As EventArgs) Handles btnSend.Click
        If comConnected Then
            mySerialPort.Write(txtInput.Text)
        End If
    End Sub

    Function ReceiveSerialData() As String
        Dim Incoming As String
        Try
            Incoming = mySerialPort.ReadExisting()
            If Incoming Is String.Empty Then
                txtOutput.Text &= "nothing" & vbCrLf
                'Return "nothing" & vbCrLf
            Else
                txtOutput.Text &= Incoming
                'Return Incoming
            End If
        Catch ex As Exception
            MessageBox.Show(ex.Message)
        End Try
    End Function

    Function SendSerialData(ByVal Outcoming As String) As String
        mySerialPort.Write(Outcoming)
    End Function

    Public Sub spDataReceivedTrigger(ByVal sender As Object, ByVal e As System.IO.Ports.SerialDataReceivedEventArgs) Handles mySerialPort.DataReceived
        Me.Invoke(New EventHandler(AddressOf ReceiveSerialData)) '调用接收数据函数   
    End Sub

    Private Sub ckbTopMost_CheckedChanged(sender As Object, e As EventArgs) Handles ckbTopMost.CheckedChanged
        If ckbTopMost.Checked Then
            Me.TopMost = True
        Else
            Me.TopMost = False
        End If
    End Sub

    Private Function addComToList() As String

        Dim intIndex As Integer
        cboCOMPort.Items.Clear()

        For Each sp As String In My.Computer.Ports.SerialPortNames
            intIndex = cboCOMPort.Items.Add(sp)
            If String.Equals(sp, My.Computer.Registry.GetValue(c_strCP2102, "PortName", "")) Then
                cboCOMPort.SelectedIndex = intIndex
                comDetected = True
                comPORT = cboCOMPort.SelectedItem
            End If
        Next

        If String.Equals(comPORT, "") Then
            cboCOMPort.SelectedIndex = 0
            comDetected = False
            comPORT = cboCOMPort.SelectedItem
        End If

        Return comPORT
    End Function

    Private Function checkComPort(ByVal s As String) As Boolean
        For Each sp As String In My.Computer.Ports.SerialPortNames
            If String.Equals(sp, s) Then
                Return True
            End If
        Next
        Return False
    End Function

    Private Sub resetCboList()

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

    Private Sub btnExcel_Click(sender As Object, e As EventArgs) Handles btnExcel.Click

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


End Class
