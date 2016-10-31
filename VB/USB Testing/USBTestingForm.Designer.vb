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
        Me.lblOutput = New System.Windows.Forms.Label()
        Me.txtOutput = New System.Windows.Forms.TextBox()
        Me.btnClear = New System.Windows.Forms.Button()
        Me.lblCOMPort = New System.Windows.Forms.Label()
        Me.btnConnect = New System.Windows.Forms.Button()
        Me.cboCOMPort = New System.Windows.Forms.ComboBox()
        Me.txtInput = New System.Windows.Forms.TextBox()
        Me.lblInput = New System.Windows.Forms.Label()
        Me.btnSend = New System.Windows.Forms.Button()
        Me.ckbTopMost = New System.Windows.Forms.CheckBox()
        Me.btnExcel = New System.Windows.Forms.Button()
        Me.DataGridView1 = New System.Windows.Forms.DataGridView()
        CType(Me.DataGridView1, System.ComponentModel.ISupportInitialize).BeginInit()
        Me.SuspendLayout()
        '
        'lblOutput
        '
        Me.lblOutput.AutoSize = True
        Me.lblOutput.Location = New System.Drawing.Point(210, 21)
        Me.lblOutput.Name = "lblOutput"
        Me.lblOutput.Size = New System.Drawing.Size(41, 12)
        Me.lblOutput.TabIndex = 0
        Me.lblOutput.Text = "Output"
        '
        'txtOutput
        '
        Me.txtOutput.Location = New System.Drawing.Point(212, 36)
        Me.txtOutput.Multiline = True
        Me.txtOutput.Name = "txtOutput"
        Me.txtOutput.ScrollBars = System.Windows.Forms.ScrollBars.Vertical
        Me.txtOutput.Size = New System.Drawing.Size(264, 346)
        Me.txtOutput.TabIndex = 1
        '
        'btnClear
        '
        Me.btnClear.Location = New System.Drawing.Point(212, 388)
        Me.btnClear.Name = "btnClear"
        Me.btnClear.Size = New System.Drawing.Size(75, 23)
        Me.btnClear.TabIndex = 2
        Me.btnClear.Text = "Clear"
        Me.btnClear.UseVisualStyleBackColor = True
        '
        'lblCOMPort
        '
        Me.lblCOMPort.AutoSize = True
        Me.lblCOMPort.Location = New System.Drawing.Point(12, 21)
        Me.lblCOMPort.Name = "lblCOMPort"
        Me.lblCOMPort.Size = New System.Drawing.Size(53, 12)
        Me.lblCOMPort.TabIndex = 3
        Me.lblCOMPort.Text = "COM Port"
        '
        'btnConnect
        '
        Me.btnConnect.Location = New System.Drawing.Point(131, 33)
        Me.btnConnect.Name = "btnConnect"
        Me.btnConnect.Size = New System.Drawing.Size(75, 23)
        Me.btnConnect.TabIndex = 4
        Me.btnConnect.Text = "Connect"
        Me.btnConnect.UseVisualStyleBackColor = True
        '
        'cboCOMPort
        '
        Me.cboCOMPort.FormattingEnabled = True
        Me.cboCOMPort.Location = New System.Drawing.Point(14, 36)
        Me.cboCOMPort.Name = "cboCOMPort"
        Me.cboCOMPort.Size = New System.Drawing.Size(111, 20)
        Me.cboCOMPort.Sorted = True
        Me.cboCOMPort.TabIndex = 5
        '
        'txtInput
        '
        Me.txtInput.Location = New System.Drawing.Point(14, 136)
        Me.txtInput.MaxLength = 10
        Me.txtInput.Name = "txtInput"
        Me.txtInput.Size = New System.Drawing.Size(192, 21)
        Me.txtInput.TabIndex = 7
        '
        'lblInput
        '
        Me.lblInput.AutoSize = True
        Me.lblInput.Location = New System.Drawing.Point(12, 121)
        Me.lblInput.Name = "lblInput"
        Me.lblInput.Size = New System.Drawing.Size(35, 12)
        Me.lblInput.TabIndex = 8
        Me.lblInput.Text = "Input"
        '
        'btnSend
        '
        Me.btnSend.Location = New System.Drawing.Point(131, 163)
        Me.btnSend.Name = "btnSend"
        Me.btnSend.Size = New System.Drawing.Size(75, 23)
        Me.btnSend.TabIndex = 9
        Me.btnSend.Text = "Send"
        Me.btnSend.UseVisualStyleBackColor = True
        '
        'ckbTopMost
        '
        Me.ckbTopMost.AutoSize = True
        Me.ckbTopMost.Location = New System.Drawing.Point(140, 192)
        Me.ckbTopMost.Name = "ckbTopMost"
        Me.ckbTopMost.Size = New System.Drawing.Size(66, 16)
        Me.ckbTopMost.TabIndex = 10
        Me.ckbTopMost.Text = "TopMost"
        Me.ckbTopMost.UseVisualStyleBackColor = True
        '
        'btnExcel
        '
        Me.btnExcel.Location = New System.Drawing.Point(302, 388)
        Me.btnExcel.Name = "btnExcel"
        Me.btnExcel.Size = New System.Drawing.Size(75, 23)
        Me.btnExcel.TabIndex = 11
        Me.btnExcel.Text = "Save Excel"
        Me.btnExcel.UseVisualStyleBackColor = True
        '
        'DataGridView1
        '
        Me.DataGridView1.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize
        Me.DataGridView1.Location = New System.Drawing.Point(530, 36)
        Me.DataGridView1.Name = "DataGridView1"
        Me.DataGridView1.RowTemplate.Height = 23
        Me.DataGridView1.Size = New System.Drawing.Size(458, 346)
        Me.DataGridView1.TabIndex = 12
        '
        'USBTestingForm
        '
        Me.AutoScaleDimensions = New System.Drawing.SizeF(6.0!, 12.0!)
        Me.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font
        Me.ClientSize = New System.Drawing.Size(1051, 443)
        Me.Controls.Add(Me.DataGridView1)
        Me.Controls.Add(Me.btnExcel)
        Me.Controls.Add(Me.ckbTopMost)
        Me.Controls.Add(Me.btnSend)
        Me.Controls.Add(Me.lblInput)
        Me.Controls.Add(Me.txtInput)
        Me.Controls.Add(Me.cboCOMPort)
        Me.Controls.Add(Me.btnConnect)
        Me.Controls.Add(Me.lblCOMPort)
        Me.Controls.Add(Me.btnClear)
        Me.Controls.Add(Me.txtOutput)
        Me.Controls.Add(Me.lblOutput)
        Me.Name = "USBTestingForm"
        Me.Text = "USB Testing"
        CType(Me.DataGridView1, System.ComponentModel.ISupportInitialize).EndInit()
        Me.ResumeLayout(False)
        Me.PerformLayout()

    End Sub

    Friend WithEvents lblOutput As Label
    Friend WithEvents txtOutput As TextBox
    Friend WithEvents btnClear As Button
    Friend WithEvents lblCOMPort As Label
    Friend WithEvents btnConnect As Button
    Friend WithEvents cboCOMPort As ComboBox
    Friend WithEvents txtInput As TextBox
    Friend WithEvents lblInput As Label
    Friend WithEvents btnSend As Button
    Friend WithEvents ckbTopMost As CheckBox
    Friend WithEvents btnExcel As Button
    Friend WithEvents DataGridView1 As DataGridView
End Class
