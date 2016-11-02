<Global.Microsoft.VisualBasic.CompilerServices.DesignerGenerated()>
Partial Class USBTestingForm
    Inherits System.Windows.Forms.Form

    'Form 重写 Dispose，以清理组件列表。
    <System.Diagnostics.DebuggerNonUserCode()>
    Protected Overrides Sub Dispose(ByVal disposing As Boolean)
        Try
            If disposing AndAlso components IsNot Nothing Then
                components.Dispose()
            End If
        Finally
            MyBase.Dispose(disposing)
        End Try
    End Sub

    'Windows 窗体设计器所必需的
    Private components As System.ComponentModel.IContainer

    '注意: 以下过程是 Windows 窗体设计器所必需的
    '可以使用 Windows 窗体设计器修改它。  
    '不要使用代码编辑器修改它。
    <System.Diagnostics.DebuggerStepThrough()>
    Private Sub InitializeComponent()
        Me.components = New System.ComponentModel.Container()
        Dim DataGridViewCellStyle1 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle2 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle3 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle10 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle11 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle4 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle5 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle6 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle7 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle8 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Dim DataGridViewCellStyle9 As System.Windows.Forms.DataGridViewCellStyle = New System.Windows.Forms.DataGridViewCellStyle()
        Me.btnClear = New System.Windows.Forms.Button()
        Me.lblCOMPort = New System.Windows.Forms.Label()
        Me.btnConnect = New System.Windows.Forms.Button()
        Me.cboCOMPort = New System.Windows.Forms.ComboBox()
        Me.ckbTopMost = New System.Windows.Forms.CheckBox()
        Me.btnExport = New System.Windows.Forms.Button()
        Me.btnEngMode = New System.Windows.Forms.Button()
        Me.cboFilterDevID = New System.Windows.Forms.ComboBox()
        Me.lblConnectionStatus = New System.Windows.Forms.Label()
        Me.btnReadNVM = New System.Windows.Forms.Button()
        Me.btnDelDevID = New System.Windows.Forms.Button()
        Me.timerRecCONNREQ = New System.Windows.Forms.Timer(Me.components)
        Me.btnStartRec = New System.Windows.Forms.Button()
        Me.txtDevIDStat = New System.Windows.Forms.TextBox()
        Me.lblNumOfDevID = New System.Windows.Forms.Label()
        Me.btnDebugSend = New System.Windows.Forms.Button()
        Me.txtDebugInput = New System.Windows.Forms.TextBox()
        Me.Label1 = New System.Windows.Forms.Label()
        Me.TextBox1 = New System.Windows.Forms.TextBox()
        Me.dgvDevIDList = New System.Windows.Forms.DataGridView()
        Me.dgvRecData = New System.Windows.Forms.DataGridView()
        Me.dgvRecDataColDevID = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.dgvRecDataColDate = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.dgvRecDataColTime = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.dgvRecDataColTemp = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.dgvRecDatacolHumi = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.dgvRecDatacolPress = New System.Windows.Forms.DataGridViewTextBoxColumn()
        Me.btnSearch = New System.Windows.Forms.Button()
        Me.btnAddDevID = New System.Windows.Forms.Button()
        Me.timerSearchDevID = New System.Windows.Forms.Timer(Me.components)
        Me.timerStatusCheck = New System.Windows.Forms.Timer(Me.components)
        Me.lblSimRecData = New System.Windows.Forms.Label()
        Me.txtSim = New System.Windows.Forms.TextBox()
        Me.btnSimSend = New System.Windows.Forms.Button()
        Me.nudONInterval = New System.Windows.Forms.NumericUpDown()
        Me.timerGetSensorData = New System.Windows.Forms.Timer(Me.components)
        Me.timerMeshOFF = New System.Windows.Forms.Timer(Me.components)
        Me.lblONInt = New System.Windows.Forms.Label()
        Me.nudOFFInterval = New System.Windows.Forms.NumericUpDown()
        Me.lblOFFInt = New System.Windows.Forms.Label()
        Me.Label2 = New System.Windows.Forms.Label()
        Me.lblSearchSecond = New System.Windows.Forms.Label()
        Me.nudSearchSecond = New System.Windows.Forms.NumericUpDown()
        CType(Me.dgvDevIDList, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.dgvRecData, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.nudONInterval, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.nudOFFInterval, System.ComponentModel.ISupportInitialize).BeginInit()
        CType(Me.nudSearchSecond, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.SuspendLayout()
        '
        'btnClear
        '
        Me.btnClear.Location = New System.Drawing.Point(689, 436)
        Me.btnClear.Margin = New System.Windows.Forms.Padding(4)
        Me.btnClear.Name = "btnClear"
        Me.btnClear.Size = New System.Drawing.Size(100, 29)
        Me.btnClear.TabIndex = 2
        Me.btnClear.Text = "Clear"
        Me.btnClear.UseVisualStyleBackColor = True
        '
        'lblCOMPort
        '
        Me.lblCOMPort.AutoSize = True
        Me.lblCOMPort.Location = New System.Drawing.Point(1, 11)
        Me.lblCOMPort.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.lblCOMPort.Name = "lblCOMPort"
        Me.lblCOMPort.Size = New System.Drawing.Size(67, 15)
        Me.lblCOMPort.TabIndex = 3
        Me.lblCOMPort.Text = "COM Port"
        '
        'btnConnect
        '
        Me.btnConnect.Location = New System.Drawing.Point(180, 26)
        Me.btnConnect.Margin = New System.Windows.Forms.Padding(4)
        Me.btnConnect.Name = "btnConnect"
        Me.btnConnect.Size = New System.Drawing.Size(100, 29)
        Me.btnConnect.TabIndex = 4
        Me.btnConnect.Text = "Connect"
        Me.btnConnect.UseVisualStyleBackColor = True
        '
        'cboCOMPort
        '
        Me.cboCOMPort.FormattingEnabled = True
        Me.cboCOMPort.Location = New System.Drawing.Point(4, 30)
        Me.cboCOMPort.Margin = New System.Windows.Forms.Padding(4)
        Me.cboCOMPort.Name = "cboCOMPort"
        Me.cboCOMPort.Size = New System.Drawing.Size(147, 23)
        Me.cboCOMPort.Sorted = True
        Me.cboCOMPort.TabIndex = 5
        '
        'ckbTopMost
        '
        Me.ckbTopMost.AutoSize = True
        Me.ckbTopMost.Location = New System.Drawing.Point(1013, 6)
        Me.ckbTopMost.Margin = New System.Windows.Forms.Padding(4)
        Me.ckbTopMost.Name = "ckbTopMost"
        Me.ckbTopMost.Size = New System.Drawing.Size(81, 19)
        Me.ckbTopMost.TabIndex = 10
        Me.ckbTopMost.Text = "TopMost"
        Me.ckbTopMost.UseVisualStyleBackColor = True
        '
        'btnExport
        '
        Me.btnExport.Location = New System.Drawing.Point(579, 436)
        Me.btnExport.Margin = New System.Windows.Forms.Padding(4)
        Me.btnExport.Name = "btnExport"
        Me.btnExport.Size = New System.Drawing.Size(100, 29)
        Me.btnExport.TabIndex = 13
        Me.btnExport.Text = "Export"
        Me.btnExport.UseVisualStyleBackColor = True
        '
        'btnEngMode
        '
        Me.btnEngMode.Location = New System.Drawing.Point(569, 849)
        Me.btnEngMode.Margin = New System.Windows.Forms.Padding(4)
        Me.btnEngMode.Name = "btnEngMode"
        Me.btnEngMode.Size = New System.Drawing.Size(100, 29)
        Me.btnEngMode.TabIndex = 14
        Me.btnEngMode.Text = "Engg Mode"
        Me.btnEngMode.UseVisualStyleBackColor = True
        '
        'cboFilterDevID
        '
        Me.cboFilterDevID.FormattingEnabled = True
        Me.cboFilterDevID.Location = New System.Drawing.Point(409, 436)
        Me.cboFilterDevID.Margin = New System.Windows.Forms.Padding(4)
        Me.cboFilterDevID.Name = "cboFilterDevID"
        Me.cboFilterDevID.Size = New System.Drawing.Size(160, 23)
        Me.cboFilterDevID.TabIndex = 15
        '
        'lblConnectionStatus
        '
        Me.lblConnectionStatus.AutoSize = True
        Me.lblConnectionStatus.Location = New System.Drawing.Point(288, 33)
        Me.lblConnectionStatus.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.lblConnectionStatus.Name = "lblConnectionStatus"
        Me.lblConnectionStatus.Size = New System.Drawing.Size(82, 15)
        Me.lblConnectionStatus.TabIndex = 16
        Me.lblConnectionStatus.Text = "Disconnected"
        '
        'btnReadNVM
        '
        Me.btnReadNVM.Location = New System.Drawing.Point(29, 401)
        Me.btnReadNVM.Margin = New System.Windows.Forms.Padding(4)
        Me.btnReadNVM.Name = "btnReadNVM"
        Me.btnReadNVM.Size = New System.Drawing.Size(100, 29)
        Me.btnReadNVM.TabIndex = 19
        Me.btnReadNVM.Text = "ReadNVM"
        Me.btnReadNVM.UseVisualStyleBackColor = True
        '
        'btnDelDevID
        '
        Me.btnDelDevID.Location = New System.Drawing.Point(245, 400)
        Me.btnDelDevID.Margin = New System.Windows.Forms.Padding(4)
        Me.btnDelDevID.Name = "btnDelDevID"
        Me.btnDelDevID.Size = New System.Drawing.Size(100, 29)
        Me.btnDelDevID.TabIndex = 20
        Me.btnDelDevID.Text = "Del"
        Me.btnDelDevID.UseVisualStyleBackColor = True
        '
        'timerRecCONNREQ
        '
        Me.timerRecCONNREQ.Interval = 500
        '
        'btnStartRec
        '
        Me.btnStartRec.Location = New System.Drawing.Point(409, 6)
        Me.btnStartRec.Margin = New System.Windows.Forms.Padding(4)
        Me.btnStartRec.Name = "btnStartRec"
        Me.btnStartRec.Size = New System.Drawing.Size(100, 29)
        Me.btnStartRec.TabIndex = 22
        Me.btnStartRec.Text = "Start Rec"
        Me.btnStartRec.UseVisualStyleBackColor = True
        '
        'txtDevIDStat
        '
        Me.txtDevIDStat.Location = New System.Drawing.Point(8, 436)
        Me.txtDevIDStat.Margin = New System.Windows.Forms.Padding(4)
        Me.txtDevIDStat.MaxLength = 10
        Me.txtDevIDStat.Name = "txtDevIDStat"
        Me.txtDevIDStat.Size = New System.Drawing.Size(368, 25)
        Me.txtDevIDStat.TabIndex = 30
        '
        'lblNumOfDevID
        '
        Me.lblNumOfDevID.AutoSize = True
        Me.lblNumOfDevID.Location = New System.Drawing.Point(346, 71)
        Me.lblNumOfDevID.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.lblNumOfDevID.Name = "lblNumOfDevID"
        Me.lblNumOfDevID.Size = New System.Drawing.Size(14, 15)
        Me.lblNumOfDevID.TabIndex = 29
        Me.lblNumOfDevID.Text = "0"
        '
        'btnDebugSend
        '
        Me.btnDebugSend.Location = New System.Drawing.Point(864, 491)
        Me.btnDebugSend.Margin = New System.Windows.Forms.Padding(4)
        Me.btnDebugSend.Name = "btnDebugSend"
        Me.btnDebugSend.Size = New System.Drawing.Size(100, 29)
        Me.btnDebugSend.TabIndex = 27
        Me.btnDebugSend.Text = "Send"
        Me.btnDebugSend.UseVisualStyleBackColor = True
        '
        'txtDebugInput
        '
        Me.txtDebugInput.Location = New System.Drawing.Point(8, 491)
        Me.txtDebugInput.Margin = New System.Windows.Forms.Padding(4)
        Me.txtDebugInput.MaxLength = 10
        Me.txtDebugInput.Name = "txtDebugInput"
        Me.txtDebugInput.Size = New System.Drawing.Size(847, 25)
        Me.txtDebugInput.TabIndex = 25
        '
        'Label1
        '
        Me.Label1.AutoSize = True
        Me.Label1.Location = New System.Drawing.Point(5, 468)
        Me.Label1.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label1.Name = "Label1"
        Me.Label1.Size = New System.Drawing.Size(37, 15)
        Me.Label1.TabIndex = 26
        Me.Label1.Text = "Input"
        '
        'TextBox1
        '
        Me.TextBox1.Location = New System.Drawing.Point(8, 525)
        Me.TextBox1.Margin = New System.Windows.Forms.Padding(4)
        Me.TextBox1.Multiline = True
        Me.TextBox1.Name = "TextBox1"
        Me.TextBox1.ScrollBars = System.Windows.Forms.ScrollBars.Vertical
        Me.TextBox1.Size = New System.Drawing.Size(955, 208)
        Me.TextBox1.TabIndex = 24
        '
        'dgvDevIDList
        '
        Me.dgvDevIDList.AllowUserToAddRows = False
        Me.dgvDevIDList.AllowUserToDeleteRows = False
        Me.dgvDevIDList.AllowUserToResizeColumns = False
        Me.dgvDevIDList.AllowUserToResizeRows = False
        Me.dgvDevIDList.AutoSizeColumnsMode = System.Windows.Forms.DataGridViewAutoSizeColumnsMode.Fill
        Me.dgvDevIDList.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize
        Me.dgvDevIDList.ColumnHeadersVisible = False
        DataGridViewCellStyle1.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter
        DataGridViewCellStyle1.BackColor = System.Drawing.SystemColors.Window
        DataGridViewCellStyle1.Font = New System.Drawing.Font("SimSun", 9.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(134, Byte))
        DataGridViewCellStyle1.ForeColor = System.Drawing.SystemColors.ControlText
        DataGridViewCellStyle1.SelectionBackColor = System.Drawing.SystemColors.Highlight
        DataGridViewCellStyle1.SelectionForeColor = System.Drawing.SystemColors.HighlightText
        DataGridViewCellStyle1.WrapMode = System.Windows.Forms.DataGridViewTriState.[False]
        Me.dgvDevIDList.DefaultCellStyle = DataGridViewCellStyle1
        Me.dgvDevIDList.EnableHeadersVisualStyles = False
        Me.dgvDevIDList.ImeMode = System.Windows.Forms.ImeMode.Disable
        Me.dgvDevIDList.Location = New System.Drawing.Point(8, 100)
        Me.dgvDevIDList.Margin = New System.Windows.Forms.Padding(4)
        Me.dgvDevIDList.MultiSelect = False
        Me.dgvDevIDList.Name = "dgvDevIDList"
        Me.dgvDevIDList.ReadOnly = True
        Me.dgvDevIDList.RowHeadersVisible = False
        Me.dgvDevIDList.RowHeadersWidthSizeMode = System.Windows.Forms.DataGridViewRowHeadersWidthSizeMode.DisableResizing
        DataGridViewCellStyle2.BackColor = System.Drawing.Color.White
        DataGridViewCellStyle2.SelectionBackColor = System.Drawing.Color.WhiteSmoke
        DataGridViewCellStyle2.SelectionForeColor = System.Drawing.Color.Black
        DataGridViewCellStyle2.WrapMode = System.Windows.Forms.DataGridViewTriState.[False]
        Me.dgvDevIDList.RowsDefaultCellStyle = DataGridViewCellStyle2
        Me.dgvDevIDList.RowTemplate.Height = 23
        Me.dgvDevIDList.RowTemplate.ReadOnly = True
        Me.dgvDevIDList.ScrollBars = System.Windows.Forms.ScrollBars.Vertical
        Me.dgvDevIDList.SelectionMode = System.Windows.Forms.DataGridViewSelectionMode.CellSelect
        Me.dgvDevIDList.ShowCellErrors = False
        Me.dgvDevIDList.ShowCellToolTips = False
        Me.dgvDevIDList.ShowEditingIcon = False
        Me.dgvDevIDList.ShowRowErrors = False
        Me.dgvDevIDList.Size = New System.Drawing.Size(369, 294)
        Me.dgvDevIDList.TabIndex = 18
        '
        'dgvRecData
        '
        Me.dgvRecData.AllowUserToAddRows = False
        Me.dgvRecData.AllowUserToDeleteRows = False
        Me.dgvRecData.AllowUserToResizeColumns = False
        Me.dgvRecData.AllowUserToResizeRows = False
        Me.dgvRecData.AutoSizeColumnsMode = System.Windows.Forms.DataGridViewAutoSizeColumnsMode.ColumnHeader
        DataGridViewCellStyle3.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft
        DataGridViewCellStyle3.BackColor = System.Drawing.SystemColors.Control
        DataGridViewCellStyle3.Font = New System.Drawing.Font("Consolas", 9.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        DataGridViewCellStyle3.ForeColor = System.Drawing.SystemColors.WindowText
        DataGridViewCellStyle3.SelectionBackColor = System.Drawing.SystemColors.Highlight
        DataGridViewCellStyle3.SelectionForeColor = System.Drawing.SystemColors.HighlightText
        Me.dgvRecData.ColumnHeadersDefaultCellStyle = DataGridViewCellStyle3
        Me.dgvRecData.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize
        Me.dgvRecData.Columns.AddRange(New System.Windows.Forms.DataGridViewColumn() {Me.dgvRecDataColDevID, Me.dgvRecDataColDate, Me.dgvRecDataColTime, Me.dgvRecDataColTemp, Me.dgvRecDatacolHumi, Me.dgvRecDatacolPress})
        Me.dgvRecData.Location = New System.Drawing.Point(409, 38)
        Me.dgvRecData.Margin = New System.Windows.Forms.Padding(4)
        Me.dgvRecData.Name = "dgvRecData"
        Me.dgvRecData.ReadOnly = True
        DataGridViewCellStyle10.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft
        DataGridViewCellStyle10.BackColor = System.Drawing.SystemColors.Control
        DataGridViewCellStyle10.Font = New System.Drawing.Font("SimSun", 9.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(134, Byte))
        DataGridViewCellStyle10.ForeColor = System.Drawing.SystemColors.WindowText
        DataGridViewCellStyle10.NullValue = Nothing
        DataGridViewCellStyle10.SelectionBackColor = System.Drawing.SystemColors.Highlight
        DataGridViewCellStyle10.SelectionForeColor = System.Drawing.SystemColors.HighlightText
        DataGridViewCellStyle10.WrapMode = System.Windows.Forms.DataGridViewTriState.[True]
        Me.dgvRecData.RowHeadersDefaultCellStyle = DataGridViewCellStyle10
        Me.dgvRecData.RowHeadersVisible = False
        Me.dgvRecData.RowHeadersWidthSizeMode = System.Windows.Forms.DataGridViewRowHeadersWidthSizeMode.DisableResizing
        DataGridViewCellStyle11.Font = New System.Drawing.Font("SimSun", 9.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(134, Byte))
        Me.dgvRecData.RowsDefaultCellStyle = DataGridViewCellStyle11
        Me.dgvRecData.RowTemplate.Height = 23
        Me.dgvRecData.Size = New System.Drawing.Size(555, 391)
        Me.dgvRecData.TabIndex = 12
        '
        'dgvRecDataColDevID
        '
        Me.dgvRecDataColDevID.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.None
        Me.dgvRecDataColDevID.DataPropertyName = "DevID"
        DataGridViewCellStyle4.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft
        DataGridViewCellStyle4.Font = New System.Drawing.Font("Calibri", 9.0!, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, CType(0, Byte))
        DataGridViewCellStyle4.NullValue = Nothing
        DataGridViewCellStyle4.WrapMode = System.Windows.Forms.DataGridViewTriState.[True]
        Me.dgvRecDataColDevID.DefaultCellStyle = DataGridViewCellStyle4
        Me.dgvRecDataColDevID.HeaderText = "DevID"
        Me.dgvRecDataColDevID.Name = "dgvRecDataColDevID"
        Me.dgvRecDataColDevID.ReadOnly = True
        Me.dgvRecDataColDevID.Resizable = System.Windows.Forms.DataGridViewTriState.[False]
        Me.dgvRecDataColDevID.Width = 60
        '
        'dgvRecDataColDate
        '
        Me.dgvRecDataColDate.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.None
        Me.dgvRecDataColDate.DataPropertyName = "Date"
        DataGridViewCellStyle5.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft
        Me.dgvRecDataColDate.DefaultCellStyle = DataGridViewCellStyle5
        Me.dgvRecDataColDate.HeaderText = "Date"
        Me.dgvRecDataColDate.Name = "dgvRecDataColDate"
        Me.dgvRecDataColDate.ReadOnly = True
        Me.dgvRecDataColDate.Resizable = System.Windows.Forms.DataGridViewTriState.[False]
        Me.dgvRecDataColDate.Width = 80
        '
        'dgvRecDataColTime
        '
        Me.dgvRecDataColTime.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.None
        Me.dgvRecDataColTime.DataPropertyName = "Time"
        DataGridViewCellStyle6.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleLeft
        Me.dgvRecDataColTime.DefaultCellStyle = DataGridViewCellStyle6
        Me.dgvRecDataColTime.HeaderText = "Time"
        Me.dgvRecDataColTime.Name = "dgvRecDataColTime"
        Me.dgvRecDataColTime.ReadOnly = True
        Me.dgvRecDataColTime.Resizable = System.Windows.Forms.DataGridViewTriState.[False]
        Me.dgvRecDataColTime.Width = 60
        '
        'dgvRecDataColTemp
        '
        Me.dgvRecDataColTemp.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.None
        Me.dgvRecDataColTemp.DataPropertyName = "Temp"
        DataGridViewCellStyle7.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter
        Me.dgvRecDataColTemp.DefaultCellStyle = DataGridViewCellStyle7
        Me.dgvRecDataColTemp.HeaderText = "Temp(C)"
        Me.dgvRecDataColTemp.Name = "dgvRecDataColTemp"
        Me.dgvRecDataColTemp.ReadOnly = True
        Me.dgvRecDataColTemp.Resizable = System.Windows.Forms.DataGridViewTriState.[False]
        Me.dgvRecDataColTemp.Width = 70
        '
        'dgvRecDatacolHumi
        '
        Me.dgvRecDatacolHumi.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.None
        Me.dgvRecDatacolHumi.DataPropertyName = "Humi"
        DataGridViewCellStyle8.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter
        Me.dgvRecDatacolHumi.DefaultCellStyle = DataGridViewCellStyle8
        Me.dgvRecDatacolHumi.HeaderText = "Humi(%)"
        Me.dgvRecDatacolHumi.Name = "dgvRecDatacolHumi"
        Me.dgvRecDatacolHumi.ReadOnly = True
        Me.dgvRecDatacolHumi.Resizable = System.Windows.Forms.DataGridViewTriState.[False]
        Me.dgvRecDatacolHumi.Width = 70
        '
        'dgvRecDatacolPress
        '
        Me.dgvRecDatacolPress.AutoSizeMode = System.Windows.Forms.DataGridViewAutoSizeColumnMode.Fill
        Me.dgvRecDatacolPress.DataPropertyName = "Press"
        DataGridViewCellStyle9.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter
        Me.dgvRecDatacolPress.DefaultCellStyle = DataGridViewCellStyle9
        Me.dgvRecDatacolPress.HeaderText = "P(kPa)"
        Me.dgvRecDatacolPress.Name = "dgvRecDatacolPress"
        Me.dgvRecDatacolPress.ReadOnly = True
        Me.dgvRecDatacolPress.Resizable = System.Windows.Forms.DataGridViewTriState.[False]
        '
        'btnSearch
        '
        Me.btnSearch.Location = New System.Drawing.Point(181, 64)
        Me.btnSearch.Margin = New System.Windows.Forms.Padding(4)
        Me.btnSearch.Name = "btnSearch"
        Me.btnSearch.Size = New System.Drawing.Size(73, 29)
        Me.btnSearch.TabIndex = 31
        Me.btnSearch.Text = "SEARCH"
        Me.btnSearch.UseVisualStyleBackColor = True
        '
        'btnAddDevID
        '
        Me.btnAddDevID.Location = New System.Drawing.Point(137, 400)
        Me.btnAddDevID.Margin = New System.Windows.Forms.Padding(4)
        Me.btnAddDevID.Name = "btnAddDevID"
        Me.btnAddDevID.Size = New System.Drawing.Size(100, 29)
        Me.btnAddDevID.TabIndex = 32
        Me.btnAddDevID.Text = "Add"
        Me.btnAddDevID.UseVisualStyleBackColor = True
        '
        'timerSearchDevID
        '
        Me.timerSearchDevID.Interval = 1000
        '
        'timerStatusCheck
        '
        Me.timerStatusCheck.Interval = 5000
        '
        'lblSimRecData
        '
        Me.lblSimRecData.AutoSize = True
        Me.lblSimRecData.Location = New System.Drawing.Point(13, 738)
        Me.lblSimRecData.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.lblSimRecData.Name = "lblSimRecData"
        Me.lblSimRecData.Size = New System.Drawing.Size(77, 15)
        Me.lblSimRecData.TabIndex = 34
        Me.lblSimRecData.Text = "SimRecData"
        '
        'txtSim
        '
        Me.txtSim.Location = New System.Drawing.Point(16, 761)
        Me.txtSim.Margin = New System.Windows.Forms.Padding(4)
        Me.txtSim.MaxLength = 20
        Me.txtSim.Name = "txtSim"
        Me.txtSim.Size = New System.Drawing.Size(847, 25)
        Me.txtSim.TabIndex = 33
        '
        'btnSimSend
        '
        Me.btnSimSend.Location = New System.Drawing.Point(872, 761)
        Me.btnSimSend.Margin = New System.Windows.Forms.Padding(4)
        Me.btnSimSend.Name = "btnSimSend"
        Me.btnSimSend.Size = New System.Drawing.Size(100, 29)
        Me.btnSimSend.TabIndex = 35
        Me.btnSimSend.Text = "Send"
        Me.btnSimSend.UseVisualStyleBackColor = True
        '
        'nudONInterval
        '
        Me.nudONInterval.Location = New System.Drawing.Point(4, 65)
        Me.nudONInterval.Margin = New System.Windows.Forms.Padding(4)
        Me.nudONInterval.Minimum = New Decimal(New Integer() {1, 0, 0, 0})
        Me.nudONInterval.Name = "nudONInterval"
        Me.nudONInterval.Size = New System.Drawing.Size(48, 25)
        Me.nudONInterval.TabIndex = 36
        Me.nudONInterval.Value = New Decimal(New Integer() {3, 0, 0, 0})
        '
        'timerGetSensorData
        '
        Me.timerGetSensorData.Interval = 5000
        '
        'timerMeshOFF
        '
        Me.timerMeshOFF.Interval = 1000
        '
        'lblONInt
        '
        Me.lblONInt.AutoSize = True
        Me.lblONInt.Location = New System.Drawing.Point(52, 70)
        Me.lblONInt.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.lblONInt.Name = "lblONInt"
        Me.lblONInt.Size = New System.Drawing.Size(27, 15)
        Me.lblONInt.TabIndex = 37
        Me.lblONInt.Text = "ON"
        '
        'nudOFFInterval
        '
        Me.nudOFFInterval.Location = New System.Drawing.Point(75, 65)
        Me.nudOFFInterval.Margin = New System.Windows.Forms.Padding(4)
        Me.nudOFFInterval.Minimum = New Decimal(New Integer() {1, 0, 0, 0})
        Me.nudOFFInterval.Name = "nudOFFInterval"
        Me.nudOFFInterval.Size = New System.Drawing.Size(40, 25)
        Me.nudOFFInterval.TabIndex = 38
        Me.nudOFFInterval.Value = New Decimal(New Integer() {3, 0, 0, 0})
        '
        'lblOFFInt
        '
        Me.lblOFFInt.AutoSize = True
        Me.lblOFFInt.Location = New System.Drawing.Point(116, 70)
        Me.lblOFFInt.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.lblOFFInt.Name = "lblOFFInt"
        Me.lblOFFInt.Size = New System.Drawing.Size(33, 15)
        Me.lblOFFInt.TabIndex = 39
        Me.lblOFFInt.Text = "OFF"
        '
        'Label2
        '
        Me.Label2.AutoSize = True
        Me.Label2.Location = New System.Drawing.Point(143, 70)
        Me.Label2.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.Label2.Name = "Label2"
        Me.Label2.Size = New System.Drawing.Size(31, 15)
        Me.Label2.TabIndex = 40
        Me.Label2.Text = "Min"
        '
        'lblSearchSecond
        '
        Me.lblSearchSecond.AutoSize = True
        Me.lblSearchSecond.Location = New System.Drawing.Point(314, 70)
        Me.lblSearchSecond.Margin = New System.Windows.Forms.Padding(4, 0, 4, 0)
        Me.lblSearchSecond.Name = "lblSearchSecond"
        Me.lblSearchSecond.Size = New System.Drawing.Size(12, 15)
        Me.lblSearchSecond.TabIndex = 41
        Me.lblSearchSecond.Text = "s"
        '
        'nudSearchSecond
        '
        Me.nudSearchSecond.Location = New System.Drawing.Point(262, 67)
        Me.nudSearchSecond.Margin = New System.Windows.Forms.Padding(4)
        Me.nudSearchSecond.Maximum = New Decimal(New Integer() {500, 0, 0, 0})
        Me.nudSearchSecond.Minimum = New Decimal(New Integer() {1, 0, 0, 0})
        Me.nudSearchSecond.Name = "nudSearchSecond"
        Me.nudSearchSecond.Size = New System.Drawing.Size(51, 25)
        Me.nudSearchSecond.TabIndex = 42
        Me.nudSearchSecond.Value = New Decimal(New Integer() {60, 0, 0, 0})
        '
        'USBTestingForm
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(8.0!, 15.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(975, 799)
        Me.Controls.Add(Me.nudSearchSecond)
        Me.Controls.Add(Me.lblSearchSecond)
        Me.Controls.Add(Me.Label2)
        Me.Controls.Add(Me.lblOFFInt)
        Me.Controls.Add(Me.nudOFFInterval)
        Me.Controls.Add(Me.lblONInt)
        Me.Controls.Add(Me.nudONInterval)
        Me.Controls.Add(Me.lblSimRecData)
        Me.Controls.Add(Me.txtSim)
        Me.Controls.Add(Me.btnSimSend)
        Me.Controls.Add(Me.btnAddDevID)
        Me.Controls.Add(Me.btnSearch)
        Me.Controls.Add(Me.dgvRecData)
        Me.Controls.Add(Me.TextBox1)
        Me.Controls.Add(Me.dgvDevIDList)
        Me.Controls.Add(Me.Label1)
        Me.Controls.Add(Me.txtDebugInput)
        Me.Controls.Add(Me.txtDevIDStat)
        Me.Controls.Add(Me.cboFilterDevID)
        Me.Controls.Add(Me.lblNumOfDevID)
        Me.Controls.Add(Me.btnDebugSend)
        Me.Controls.Add(Me.btnEngMode)
        Me.Controls.Add(Me.btnConnect)
        Me.Controls.Add(Me.lblCOMPort)
        Me.Controls.Add(Me.cboCOMPort)
        Me.Controls.Add(Me.lblConnectionStatus)
        Me.Controls.Add(Me.ckbTopMost)
        Me.Controls.Add(Me.btnExport)
        Me.Controls.Add(Me.btnStartRec)
        Me.Controls.Add(Me.btnDelDevID)
        Me.Controls.Add(Me.btnClear)
        Me.Controls.Add(Me.btnReadNVM)
        Me.Margin = New System.Windows.Forms.Padding(4)
        Me.Name = "USBTestingForm"
        Me.Text = "WSN1000"
        CType(Me.dgvDevIDList, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.dgvRecData, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.nudONInterval, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.nudOFFInterval, System.ComponentModel.ISupportInitialize).EndInit()
        CType(Me.nudSearchSecond, System.ComponentModel.ISupportInitialize).EndInit()
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub
    Friend WithEvents btnClear As Button
    Friend WithEvents lblCOMPort As Label
    Friend WithEvents btnConnect As Button
    Friend WithEvents cboCOMPort As ComboBox
    Friend WithEvents ckbTopMost As CheckBox
    Friend WithEvents btnExport As Button
    Friend WithEvents btnEngMode As Button
    Friend WithEvents cboFilterDevID As ComboBox
    Friend WithEvents lblConnectionStatus As Label
    Friend WithEvents btnReadNVM As Button
    Friend WithEvents btnDelDevID As Button
    Friend WithEvents timerRecCONNREQ As Timer
    Friend WithEvents btnStartRec As Button
    Friend WithEvents TextBox1 As TextBox
    Friend WithEvents btnDebugSend As Button
    Friend WithEvents txtDebugInput As TextBox
    Friend WithEvents Label1 As Label
    Friend WithEvents txtDevIDStat As TextBox
    Friend WithEvents lblNumOfDevID As Label
    Friend WithEvents dgvDevIDList As DataGridView
    Friend WithEvents dgvRecData As DataGridView
    Friend WithEvents dgvRecDataColDevID As DataGridViewTextBoxColumn
    Friend WithEvents dgvRecDataColDate As DataGridViewTextBoxColumn
    Friend WithEvents dgvRecDataColTime As DataGridViewTextBoxColumn
    Friend WithEvents dgvRecDataColTemp As DataGridViewTextBoxColumn
    Friend WithEvents dgvRecDatacolHumi As DataGridViewTextBoxColumn
    Friend WithEvents dgvRecDatacolPress As DataGridViewTextBoxColumn
    Friend WithEvents btnSearch As Button
    Friend WithEvents btnAddDevID As Button
    Friend WithEvents timerSearchDevID As Timer
    Friend WithEvents timerStatusCheck As Timer
    Friend WithEvents lblSimRecData As Label
    Friend WithEvents txtSim As TextBox
    Friend WithEvents btnSimSend As Button
    Friend WithEvents nudONInterval As NumericUpDown
    Friend WithEvents timerGetSensorData As Timer
    Friend WithEvents timerMeshOFF As Timer
    Friend WithEvents lblONInt As Label
    Friend WithEvents nudOFFInterval As NumericUpDown
    Friend WithEvents lblOFFInt As Label
    Friend WithEvents Label2 As Label
    Friend WithEvents lblSearchSecond As Label
    Friend WithEvents nudSearchSecond As NumericUpDown
End Class
