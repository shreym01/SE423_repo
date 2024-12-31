VERSION 5.00
Begin VB.Form Form1 
   Caption         =   "Final Contest 2009 Course Layout"
   ClientHeight    =   8550
   ClientLeft      =   45
   ClientTop       =   330
   ClientWidth     =   11490
   LinkTopic       =   "Form1"
   ScaleHeight     =   8550
   ScaleWidth      =   11490
   StartUpPosition =   3  'Windows Default
   Begin VB.CommandButton cmd_lock 
      Caption         =   "LOCK"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   12
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   372
      Left            =   360
      TabIndex        =   40
      Top             =   8040
      Width           =   1692
   End
   Begin VB.VScrollBar VScrollOrange 
      Height          =   480
      Left            =   3240
      Max             =   3
      TabIndex        =   36
      Top             =   6650
      Width           =   252
   End
   Begin VB.VScrollBar VScrollBlue 
      Height          =   492
      Left            =   1760
      Max             =   3
      TabIndex        =   33
      Top             =   6650
      Width           =   252
   End
   Begin VB.CommandButton btnStopWatch 
      Caption         =   "START"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   12
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   972
      Left            =   2280
      TabIndex        =   31
      Top             =   7440
      Width           =   1572
   End
   Begin VB.Timer Timer1 
      Enabled         =   0   'False
      Interval        =   250
      Left            =   4200
      Top             =   7560
   End
   Begin VB.CommandButton btnCalcScore 
      Caption         =   "CALC SCORE"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   12
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   732
      Left            =   6600
      TabIndex        =   24
      Top             =   4560
      Width           =   1692
   End
   Begin VB.TextBox txtBally 
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   288
      Index           =   4
      Left            =   9360
      TabIndex        =   22
      Text            =   "0.00"
      Top             =   4200
      Width           =   492
   End
   Begin VB.TextBox txtBally 
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   288
      Index           =   3
      Left            =   9360
      TabIndex        =   21
      Text            =   "0.00"
      Top             =   3960
      Width           =   492
   End
   Begin VB.TextBox txtBally 
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   288
      Index           =   2
      Left            =   9360
      TabIndex        =   20
      Text            =   "0.00"
      Top             =   3720
      Width           =   492
   End
   Begin VB.TextBox txtBally 
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   288
      Index           =   1
      Left            =   9360
      TabIndex        =   19
      Text            =   "0.00"
      Top             =   3480
      Width           =   492
   End
   Begin VB.TextBox txtBally 
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   288
      Index           =   0
      Left            =   9360
      TabIndex        =   18
      Text            =   "0.00"
      Top             =   3240
      Width           =   492
   End
   Begin VB.CommandButton btnBallColor 
      BackColor       =   &H000080FF&
      Caption         =   "Orange"
      Height          =   252
      Index           =   4
      Left            =   9960
      MaskColor       =   &H000080FF&
      Style           =   1  'Graphical
      TabIndex        =   17
      Top             =   4200
      Width           =   732
   End
   Begin VB.CommandButton btnBallColor 
      BackColor       =   &H000080FF&
      Caption         =   "Orange"
      Height          =   252
      Index           =   3
      Left            =   9960
      MaskColor       =   &H000080FF&
      Style           =   1  'Graphical
      TabIndex        =   16
      Top             =   3960
      Width           =   732
   End
   Begin VB.CommandButton btnBallColor 
      BackColor       =   &H000080FF&
      Caption         =   "Orange"
      Height          =   252
      Index           =   2
      Left            =   9960
      MaskColor       =   &H000080FF&
      Style           =   1  'Graphical
      TabIndex        =   15
      Top             =   3720
      Width           =   732
   End
   Begin VB.CommandButton btnBallColor 
      BackColor       =   &H000080FF&
      Caption         =   "Orange"
      Height          =   252
      Index           =   1
      Left            =   9960
      MaskColor       =   &H000080FF&
      Style           =   1  'Graphical
      TabIndex        =   14
      Top             =   3480
      Width           =   732
   End
   Begin VB.CommandButton btnBallColor 
      BackColor       =   &H000080FF&
      Caption         =   "Orange"
      Height          =   252
      Index           =   0
      Left            =   9960
      MaskColor       =   &H000080FF&
      Style           =   1  'Graphical
      TabIndex        =   13
      Top             =   3240
      Width           =   732
   End
   Begin VB.TextBox txtBallx 
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0.00"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   288
      Index           =   4
      Left            =   8760
      TabIndex        =   12
      Text            =   "0.0"
      Top             =   4200
      Width           =   492
   End
   Begin VB.TextBox txtBallx 
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0.00"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   288
      Index           =   3
      Left            =   8760
      TabIndex        =   11
      Text            =   "0.0"
      Top             =   3960
      Width           =   492
   End
   Begin VB.TextBox txtBallx 
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0.00"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   372
      Index           =   2
      Left            =   8760
      TabIndex        =   10
      Text            =   "0.0"
      Top             =   3720
      Width           =   492
   End
   Begin VB.TextBox txtBallx 
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0.00"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   288
      Index           =   1
      Left            =   8760
      TabIndex        =   9
      Text            =   "0.0"
      Top             =   3480
      Width           =   492
   End
   Begin VB.TextBox txtBallx 
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0.00"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   288
      Index           =   0
      Left            =   8760
      TabIndex        =   8
      Text            =   "0.0"
      Top             =   3240
      Width           =   492
   End
   Begin VB.PictureBox pbxArena 
      AutoRedraw      =   -1  'True
      BackColor       =   &H00FFFFFF&
      Height          =   6684
      Left            =   120
      ScaleHeight     =   6630
      ScaleWidth      =   5190
      TabIndex        =   1
      Top             =   600
      Width           =   5244
      Begin VB.Label lblFive 
         BackStyle       =   0  'Transparent
         Caption         =   "5"
         Height          =   255
         Left            =   2040
         TabIndex        =   46
         Top             =   3360
         Width           =   255
      End
      Begin VB.Label lblFour 
         BackStyle       =   0  'Transparent
         Caption         =   "4"
         Height          =   255
         Left            =   2400
         TabIndex        =   45
         Top             =   2520
         Width           =   255
      End
      Begin VB.Label lblThree 
         BackStyle       =   0  'Transparent
         Caption         =   "3"
         Height          =   255
         Left            =   2280
         TabIndex        =   44
         Top             =   1440
         Width           =   255
      End
      Begin VB.Label lblTwo 
         BackStyle       =   0  'Transparent
         Caption         =   "2"
         Height          =   255
         Left            =   1080
         TabIndex        =   43
         Top             =   1320
         Width           =   255
      End
      Begin VB.Label lblOne 
         BackStyle       =   0  'Transparent
         Caption         =   "1"
         Height          =   255
         Left            =   720
         TabIndex        =   42
         Top             =   3120
         Width           =   255
      End
      Begin VB.Label lblStart 
         BackStyle       =   0  'Transparent
         Caption         =   "Start"
         Height          =   255
         Left            =   1440
         TabIndex        =   41
         Top             =   2640
         Width           =   495
      End
      Begin VB.Label lblOrange 
         BackStyle       =   0  'Transparent
         Caption         =   "0"
         BeginProperty Font 
            Name            =   "MS Sans Serif"
            Size            =   9.75
            Charset         =   0
            Weight          =   700
            Underline       =   0   'False
            Italic          =   0   'False
            Strikethrough   =   0   'False
         EndProperty
         Height          =   372
         Left            =   3390
         TabIndex        =   35
         Top             =   6010
         Width           =   252
      End
      Begin VB.Label lblBlue 
         BackStyle       =   0  'Transparent
         Caption         =   "0"
         BeginProperty Font 
            Name            =   "MS Sans Serif"
            Size            =   9.75
            Charset         =   0
            Weight          =   700
            Underline       =   0   'False
            Italic          =   0   'False
            Strikethrough   =   0   'False
         EndProperty
         Height          =   372
         Left            =   1920
         TabIndex        =   34
         Top             =   6010
         Width           =   252
      End
   End
   Begin VB.CommandButton drawButton 
      Caption         =   "DRAW BALLS"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   12
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   612
      Left            =   360
      TabIndex        =   0
      Top             =   7440
      Width           =   1692
   End
   Begin VB.Label lblTotalScore 
      Caption         =   "Total Score =    seconds"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.75
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   375
      Left            =   7920
      TabIndex        =   39
      Top             =   5880
      Width           =   3735
   End
   Begin VB.Label Label4 
      BackStyle       =   0  'Transparent
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.75
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   372
      Left            =   0
      TabIndex        =   38
      Top             =   0
      Width           =   252
   End
   Begin VB.Label lblScoreBallDrop 
      Caption         =   "BallDrop = -0 seconds"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.75
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   375
      Left            =   8400
      TabIndex        =   37
      Top             =   4920
      Width           =   2775
   End
   Begin VB.Label lblTime 
      Caption         =   "Time:      0.0 s"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.75
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   492
      Left            =   4200
      TabIndex        =   32
      Top             =   7560
      Width           =   1692
   End
   Begin VB.Label lblScoreLocation 
      Caption         =   "Location = -0 seconds"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.75
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   375
      Left            =   8400
      TabIndex        =   30
      Top             =   4560
      Width           =   2775
   End
   Begin VB.Label lblDist 
      Caption         =   "dist"
      Height          =   255
      Index           =   4
      Left            =   10800
      TabIndex        =   29
      Top             =   4200
      Width           =   495
   End
   Begin VB.Label lblDist 
      Caption         =   "dist"
      Height          =   255
      Index           =   3
      Left            =   10800
      TabIndex        =   28
      Top             =   3960
      Width           =   495
   End
   Begin VB.Label lblDist 
      Caption         =   "dist"
      Height          =   255
      Index           =   2
      Left            =   10800
      TabIndex        =   27
      Top             =   3720
      Width           =   495
   End
   Begin VB.Label lblDist 
      Caption         =   "dist"
      Height          =   255
      Index           =   1
      Left            =   10800
      TabIndex        =   26
      Top             =   3480
      Width           =   495
   End
   Begin VB.Label lblDist 
      Caption         =   "dist"
      BeginProperty DataFormat 
         Type            =   1
         Format          =   "0.00"
         HaveTrueFalseNull=   0
         FirstDayOfWeek  =   0
         FirstWeekOfYear =   0
         LCID            =   1033
         SubFormatType   =   1
      EndProperty
      Height          =   255
      Index           =   0
      Left            =   10800
      TabIndex        =   25
      Top             =   3240
      Width           =   495
   End
   Begin VB.Line Line1 
      Index           =   4
      X1              =   7920
      X2              =   8640
      Y1              =   4320
      Y2              =   4320
   End
   Begin VB.Line Line1 
      Index           =   3
      X1              =   7920
      X2              =   8640
      Y1              =   4080
      Y2              =   4080
   End
   Begin VB.Line Line1 
      Index           =   2
      X1              =   7920
      X2              =   8640
      Y1              =   3840
      Y2              =   3840
   End
   Begin VB.Line Line1 
      Index           =   1
      X1              =   7920
      X2              =   8640
      Y1              =   3600
      Y2              =   3600
   End
   Begin VB.Line Line1 
      Index           =   0
      X1              =   7920
      X2              =   8640
      Y1              =   3360
      Y2              =   3360
   End
   Begin VB.Label Label2 
      Caption         =   "Ball     X       Y      Color   Dist"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   9.75
         Charset         =   0
         Weight          =   700
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      Height          =   255
      Left            =   8280
      TabIndex        =   23
      Top             =   3000
      Width           =   3015
   End
   Begin VB.Label Label3 
      Alignment       =   2  'Center
      BackColor       =   &H8000000E&
      BorderStyle     =   1  'Fixed Single
      Caption         =   "RANDOM BALL POSITIONER"
      BeginProperty Font 
         Name            =   "MS Sans Serif"
         Size            =   13.5
         Charset         =   0
         Weight          =   400
         Underline       =   0   'False
         Italic          =   0   'False
         Strikethrough   =   0   'False
      EndProperty
      ForeColor       =   &H00FF8080&
      Height          =   372
      Left            =   720
      TabIndex        =   7
      Top             =   120
      Width           =   4092
   End
   Begin VB.Label lblBall 
      Caption         =   "Ball#, ( x, y), color"
      Height          =   255
      Index           =   4
      Left            =   6000
      TabIndex        =   6
      Top             =   4200
      Width           =   1935
   End
   Begin VB.Label lblBall 
      Caption         =   "Ball#, ( x, y), color"
      Height          =   255
      Index           =   3
      Left            =   6000
      TabIndex        =   5
      Top             =   3960
      Width           =   1935
   End
   Begin VB.Label lblBall 
      Caption         =   "Ball#, ( x, y), color"
      Height          =   255
      Index           =   2
      Left            =   6000
      TabIndex        =   4
      Top             =   3720
      Width           =   1935
   End
   Begin VB.Label lblBall 
      Caption         =   "Ball#, ( x, y), color"
      Height          =   255
      Index           =   1
      Left            =   6000
      TabIndex        =   3
      Top             =   3480
      Width           =   1935
   End
   Begin VB.Label lblBall 
      Caption         =   "Ball#, ( x, y), color"
      Height          =   255
      Index           =   0
      Left            =   6000
      TabIndex        =   2
      Top             =   3240
      Width           =   1935
   End
End
Attribute VB_Name = "Form1"
Attribute VB_GlobalNameSpace = False
Attribute VB_Creatable = False
Attribute VB_PredeclaredId = True
Attribute VB_Exposed = False
Dim ballCount As Integer
Dim ballDrawn As Integer
Dim scoreBallLoc, scoreCanAccr, scoreBallDrop, scoreLost As Integer
Dim radius As Double
Dim angle  As Double
Dim TimerTime As Double
Dim xBall  As Double
Dim yBall  As Double
Dim CORRECTLY_IDEN_DIST As Double
Dim xGlobalBall  As Double
Dim yGlobalBall  As Double
Dim gbAreaXs(), gbAreaYs(), gbAreaXe(), gbAreaYe() As Variant
Dim ballX(), ballY(), ballC() As Variant
Dim obstacleX(), obstacleY(), obstacleNum() As Variant
Dim obAreaCenterX(), obAreaCenterY() As Variant

Dim xf, yf, xoffset, yoffset As Integer
Dim orangeGBColor, blueGBColor As Long
     


Private Sub btnBallColor_Click(Index As Integer)
'Change the color of the button
    If btnBallColor(Index).BackColor = &H80FF& Then
        btnBallColor(Index).BackColor = &HFFFF00
        btnBallColor(Index).Caption = "Blue"
        Else
        btnBallColor(Index).BackColor = &H80FF&
        btnBallColor(Index).Caption = "Orange"
        End If

End Sub

Private Sub btnCalcScore_Click()
'associate 'discovered' balls with the balls on the course
Dim minDist(), ballminDist(), ballConnect() As Variant
' initialize variables
    minDist = Array(999, 999, 999, 999, 999)
    ballminDist = Array(999, 999, 999, 999, 999)
    ballConnect = Array(999, 999, 999, 999, 999)
    For I = 0 To 4
        Line1(I).X1 = txtBallx(I).Left
        Line1(I).Y1 = txtBallx(I).Top + 0.5 * txtBallx(I).Height
        Line1(I).X2 = txtBallx(I).Left
        Line1(I).Y2 = txtBallx(I).Top + 0.5 * txtBallx(I).Height
    Next
'calculate the score for the course
For N = 0 To 4 'outer loop to try to attach each ball
    For I = 0 To 4  'each ball found
        For J = 0 To 4 ' each ball in the course
            'first see if colors match
            If (btnBallColor(I).Caption = "Blue" And ballC(J) = 1) Or (btnBallColor(I).Caption = "Orange" And ballC(J) = 0) Then
                'calculate the distance
                dist = ((CDbl(txtBallx(I).Text) - ballX(J)) ^ 2 + (CDbl(txtBally(I).Text) - ballY(J)) ^ 2) ^ 0.5
                If dist < minDist(I) Then
                    If dist < ballminDist(J) Then
                        If ballminDist(J) < 999 Then
                            'disconnect that ball
                            M = ballConnect(J)
                            minDist(M) = 999
                            'draw the disconnection
                            Line1(M).X1 = txtBallx(M).Left
                            Line1(M).Y1 = txtBallx(M).Top + 0.5 * txtBallx(M).Height
                        End If
                        'disconnect this ball if it was connected
                        For M = 0 To 4
                            If ballConnect(M) = I Then
                                ballConnect(M) = 999
                                ballminDist(M) = 999
                            End If
                        Next
                        ballminDist(J) = dist
                        minDist(I) = dist
                        ballConnect(J) = I 'connect this ball
                        Line1(I).X1 = lblBall(J).Left + lblBall(J).Width
                        Line1(I).Y1 = lblBall(J).Top + 0.5 * lblBall(J).Height
                        Line1(I).X2 = txtBallx(I).Left
                        Line1(I).Y2 = txtBallx(I).Top + 0.5 * txtBallx(I).Height
                        
                    End If
                End If
            End If
        Next
    Next
    Next
    scoreBallLoc = 0
    For I = 0 To 4
        If minDist(I) = 999 Then
            lblDist(I) = "INF"
        Else
            lblDist(I) = Format(minDist(I), "##0.00")
        End If
        If minDist(I) < CORRECTLY_IDEN_DIST Then
            lblDist(I).Font.Bold = True
            scoreBallLoc = scoreBallLoc - 20
            Line1(I).BorderWidth = 4
            Line1(I).BorderColor = &HFFF&
        Else
            lblDist(I).Font.Bold = False
            Line1(I).BorderWidth = 1
            Line1(I).BorderColor = &H0&
        End If
    Next
    
    If (VScrollBlue.Value + VScrollOrange.Value) > 2 Then
        scoreBallDrop = -20 * (VScrollBlue.Value + VScrollOrange.Value - 2)
    Else
        scoreBallDrop = 0
    End If
    
    scoreCanAccr = 0
    lblScoreLocation = "Location = " + CStr(scoreBallLoc) + " seconds"
    lblScoreBallDrop = "BallDrop = " + CStr(scoreBallDrop) + " seconds"
    'lblScoreCanAccr = "CanAccr = " + CStr(scoreCanAccr) + " seconds"
    'lblscoreLost = "Found Way Back = " + CStr(scoreLost) + "seconds"
    ScoreTotal = TimerTime + scoreBallLoc + scoreCanAccr + scoreBallDrop
    lblTotalScore = "Total Score = " + Format(ScoreTotal, "0.00") + " seconds"
End Sub

Private Sub btnStopWatch_Click()
'function to run a stop watch
    If btnStopWatch.Caption = "START" Then
        btnStopWatch.Caption = "STOP"
        TimerTime = 0#
        lblTime = "Time:     " + Format(TimerTime, "#0.00s")
        Timer1.Enabled = True
    Else
        btnStopWatch.Caption = "START"
        Timer1.Enabled = False
    End If
End Sub

Private Sub cmd_lock_Click()
    If cmd_lock.Caption = "LOCK" Then
        drawButton.Enabled = False
        cmd_lock.Caption = "UNLOCK"
    Else
        drawButton.Enabled = True
        cmd_lock.Caption = "LOCK"
    End If
    
End Sub

Private Sub Command1_Click()
    
End Sub

Private Sub cmd_Lost_Click()
    If cmd_Lost.Caption = "NO" Then
        cmd_Lost.Caption = "YES"
    Else
        cmd_Lost.Caption = "NO"
    End If
    
End Sub

Private Sub drawButton_Click()
    pbxArena.Cls
    Dim samebox As Boolean
    Dim collision As Boolean
        Dim box As Integer
    ' balls must be more than minDist foot apart.
    Dim minDist As Double
    Dim minobstacleDist As Double
    minobstacleDist = 2.5
    minDist = 1.5
    
    samebox = True
    While samebox
        obstacleNum(0) = Rand(0, 29)
        
        If obstacleNum(0) <> 7 And obstacleNum(0) <> 10 Then
            samebox = False
        End If

    Wend
    
    For I = 1 To 4
        samebox = True
        While samebox
            obstacleNum(I) = Rand(0, 29)
            samebox = False
            For J = 0 To I - 1
                If obstacleNum(I) = obstacleNum(J) Or obstacleNum(I) = 7 Or obstacleNum(I) = 10 Then
                    samebox = True
                End If
            Next J
        Wend
    Next I
    
    'Draw Obstacles
    For I = 0 To 4
        pbxArena.FillColor = vbRed
        pbxArena.Line (xoffset + xf * obstacleX(CInt(obstacleNum(I))) - xf * 1, yoffset - yf * obstacleY(CInt(obstacleNum(I))) + yf * 1)-(xoffset + xf * obstacleX(CInt(obstacleNum(I))) + xf * 1, yoffset - yf * obstacleY(CInt(obstacleNum(I))) + yf * 1), vbRed
        pbxArena.Line (xoffset + xf * obstacleX(CInt(obstacleNum(I))) + xf * 1, yoffset - yf * obstacleY(CInt(obstacleNum(I))) + yf * 1)-(xoffset + xf * obstacleX(CInt(obstacleNum(I))) + xf * 1, yoffset - yf * obstacleY(CInt(obstacleNum(I))) - yf * 1), vbRed
        pbxArena.Line (xoffset + xf * obstacleX(obstacleNum(I)) + xf * 1, yoffset - yf * obstacleY(obstacleNum(I)) - yf * 1)-(xoffset + xf * obstacleX(obstacleNum(I)) - xf * 1, yoffset - yf * obstacleY(obstacleNum(I)) - yf * 1), vbRed
        pbxArena.Line (xoffset + xf * obstacleX(obstacleNum(I)) - xf * 1, yoffset - yf * obstacleY(obstacleNum(I)) - yf * 1)-(xoffset + xf * obstacleX(obstacleNum(I)) - xf * 1, yoffset - yf * obstacleY(obstacleNum(I)) + yf * 1), vbRed
    Next I
    For I = 0 To 4
        'Choose color!
        'for 1 and 2, place blue balls
        ' for 3 and 4, place orange balls
        ' ball 5 is either orange or blue
        If I < 2 Then
            ballC(I) = 1 '1 means blue
        ElseIf I < 4 Then
            ballC(I) = 0
        Else: ballC(I) = Rand(0, 1)
        End If
        
        ' place ball
        collision = True
        While collision
           'pick which box
           box = Rand(0, 4)
            'pick a location  gbAreaXs(), gbAreaYs(), gbAreaXe(), gbAreaYe()
            ballX(I) = gbAreaXs(box) + Rand(0, 2 * (gbAreaXe(box) - gbAreaXs(box))) / 2
            ballY(I) = gbAreaYs(box) + Rand(0, 2 * (gbAreaYe(box) - gbAreaYs(box))) / 2
            
            ' check for collision
            collision = False
            For J = 0 To I - 1
               If (ballX(I) - ballX(J)) ^ 2 + (ballY(I) - ballY(J)) ^ 2 < minDist ^ 2 Then
                collision = True
               End If
            Next
            ' check for obstacle collision
            For K = 0 To 4
                If (ballX(I) - obstacleX(obstacleNum(K))) ^ 2 + (ballY(I) - obstacleY(obstacleNum(K))) ^ 2 < 2.5 ^ 2 Then
                    collision = True
                End If
            Next K
        Wend
        Dim color As Variant
        'draw the ball on the course
        If ballC(I) > 0 Then
            pbxArena.FillColor = blueGBColor
            pbxArena.Circle (xoffset + xf * ballX(I), yoffset - yf * ballY(I)), xf * 0.15, blueGBColor
            pbxArena.Circle (xoffset + xf * ballX(I), yoffset - yf * ballY(I)), xf * 0.1, blueGBColor
            pbxArena.Circle (xoffset + xf * ballX(I), yoffset - yf * ballY(I)), xf * 0.05, blueGBColor
            color = "blue       "
        Else
            pbxArena.FillColor = orangeGBColor
            pbxArena.Circle (xoffset + xf * ballX(I), yoffset - yf * ballY(I)), xf * 0.15, orangeGBColor
            pbxArena.Circle (xoffset + xf * ballX(I), yoffset - yf * ballY(I)), xf * 0.1, orangeGBColor
            pbxArena.Circle (xoffset + xf * ballX(I), yoffset - yf * ballY(I)), xf * 0.05, orangeGBColor
            color = "orange "
        End If
        'pbxArena.Print ("(" + CStr(ballX(I)) + ", " + CStr(ballY(I)) + ")")
        
        lblBall(I) = "Ball " + CStr(I) + ", " + color + "(" + CStr(ballX(I)) + ", " + CStr(ballY(I)) + ")"
    Next
    
    
    ' place obstacle
'    collision = True
'    While collision
       'pick which box
'       box = Rand(0, 4)
       'pick a location  gbAreaXs(), gbAreaYs(), gbAreaXe(), gbAreaYe()
'       obstacleX = gbAreaXs(box) + Rand(0, 4 * (gbAreaXe(box) - gbAreaXs(box))) / 4
'       obstacleY = gbAreaYs(box) + Rand(0, 4 * (gbAreaYe(box) - gbAreaYs(box))) / 4
        
       ' check for collision
'       collision = False
'       For J = 0 To 4
'          If (obstacleX - ballX(J)) ^ 2 + (obstacleY - ballY(J)) ^ 2 < minobstacleDist ^ 2 Then
'              collision = True
'          End If
'       Next
'    Wend
    'draw the obstacle on the course
'    pbxArena.FillColor = orangeGBColor
'    pbxArena.Circle (xoffset + xf * obstacleX, yoffset - yf * obstacleY), xf * 1, orangeGBColor
End Sub


Private Sub Form_Load()
    CORRECTLY_IDEN_DIST = 0.5 ' half tile radius
    'initialize arena
    'x = {{-5,-2},{2,5}, {-4.5,-3}, {3,4.5}, {-1,1}}
    gbAreaXs = Array(-5, 0, -5, 0, -5)
    gbAreaXe = Array(-0, 5, 0, 5, 5)
    'y = {{.5, 2},{.5,2},{4,8},{4,8},{9,11.5}}
    gbAreaYs = Array(1, 1, 4, 4, 9)
     gbAreaYe = Array(4, 4, 9, 9, 11#)
     orangeGBColor = RGB(255, 102, 0) ' orange golf ball
    blueGBColor = RGB(0, 153, 255) '  blue golf ball
    'draw arena
    DrawArena
    'initialize more variables
    ballX() = Array(0, 0, 0, 0, 0)
    ballY() = Array(0, 0, 0, 0, 0)
    ballC() = Array(0, 0, 0, 0, 0)
    
    obstacleX() = Array(-5, -3, -1, 1, 3, 5, -5, -3, -1, 1, 3, 5, -5, -3, -1, 1, 3, 5, -5, -3, -1, 1, 3, 5, -5, -3, -1, 1, 3, 5)
    obstacleY() = Array(9, 9, 9, 9, 9, 9, 7, 7, 7, 7, 7, 7, 5, 5, 5, 5, 5, 5, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1, 1, 1)
    obstacleNum() = Array(0, 0, 0, 0, 0)
    
    ' Initialize variables
    ballCount = 0
    
    'Initialize the random # generator.
    Randomize
    
    'Draw the course
    drawButton_Click
    
    
End Sub

'===============================================
'Rand - Return a random number in a given range.
'
'Parameters:
'  Low  - The lower bounds of the range.
'  High - The upper bounds of the range.
'
'Returns:
'  Returns a random number from Low..High.
'===============================================

Public Function Rand(ByVal Low As Double, _
                     ByVal High As Double) As Double
  Rand = Int((High - Low + 1) * Rnd) + Low
End Function

'used to draw the arena
Private Sub DrawArena()
    Dim doorIncluded As Integer
    doorIncluded = 0
    ' set transform between screen and workspace coordinates
    TilesY = 18
    TilesX = 14
    TileDimension = pbxArena.ScaleWidth / TilesX
    pbxArena.Height = TileDimension * TilesY
    
    ' draw lines on arena
    Dim xs, ys, I, ytemp, xtemp As Integer
    
    xs = 0
    xf = pbxArena.ScaleWidth / TilesX
    ys = 0
    yf = pbxArena.ScaleHeight / TilesY
    xtemp = 7
    xoffset = xf * xtemp
    ytemp = 13
    yoffset = yf * ytemp
    
    For I = 0 To TilesY + 1
        'If (I < TilesX + 1) Then
            'Tile grid Lines
            pbxArena.Line (xf * TilesY, yf * I)-(xs, yf * I), QBColor(8) 'Horizontal Lines
            'Astar grid Lines
            'pbxArena.Line (xf * TilesY, 0.5 * yf + yf * I)-(xs, 0.5 * yf + yf * I), QBColor(13) 'Horizontal Lines
            
            pbxArena.CurrentX = xs
            pbxArena.CurrentY = yf * I
            pbxArena.Print CStr(ytemp - I)
        'End If
        'If (I < TilesY + 1) Then
            'Tile grid Lines
            pbxArena.Line (xf * I, ys)-(xf * I, yf * TilesY), QBColor(8) 'Vertical Lines
            'Astar grid Lines
            'pbxArena.Line (xf * 0.5 + xf * I, ys)-(xf * 0.5 + xf * I, yf * TilesY), QBColor(13) 'Vertical Lines

            pbxArena.CurrentX = xf * I
            pbxArena.CurrentY = xf * (TilesY - 5)
            pbxArena.Print CStr(I - xtemp)
        'End If
    Next I
    
    'pbxArena.CurrentX = xf * 2
    'pbxArena.CurrentY = xf * 19.5
    'pbxArena.Print ("(0,0)")
    
    
    'draw the walls in the arena
    Dim wallColor As Long
    pbxArena.DrawWidth = 4
    wallColor = RGB(139, 69, 19)
    If (doorIncluded) Then
        pbxArena.Line (xoffset + xf * 2, yoffset + yf * 0)-Step(xf * -8, 0), wallColor 'doorway to lower
    Else
'        pbxArena.Line (xoffset + xf * 2, yoffset + yf * 0)-Step(xf * 4# / Sqr(2), yf * 4# / Sqr(2)), wallColor '
'        pbxArena.Line (xoffset + xf * -2, yoffset + yf * 0)-Step(xf * -4# / Sqr(2), yf * 4# / Sqr(2)), wallColor '
        pbxArena.Line (xoffset + xf * -2, yoffset + yf * 0)-Step(xf * -4, 0), wallColor 'doorway to lower
    End If
    pbxArena.Line -Step(0, yf * -12), wallColor   'lower
    pbxArena.Line -Step(xf * 12, 0), wallColor  ' right
    pbxArena.Line -Step(0, yf * 12), wallColor    'upper
    pbxArena.Line -Step(xf * -4, 0), wallColor    'left to doorway
    
'    pbxArena.Line (xoffset + xf * 2, yoffset + yf * -8)-Step(0, yf * 4), wallColor 'doorway to lower
'    pbxArena.Line -Step(xf * -4, 0), wallColor    'lower
'    pbxArena.Line -Step(0, xf * -4), wallColor  ' right
    'reset the pen
    pbxArena.DrawWidth = 1
    'set color specs
    Dim canColor, lightColor, greenLandmarkColor, blueLandmarkColor, yellowLandmarkColor, refuelColor As Long
     
    canColor = RGB(255, 0, 0) ' can
    lightColor = RGB(255, 255, 255) '  light
    greenLandmarkColor = RGB(0, 255, 0) ' green landmark
    blueLandmarkColor = RGB(0, 0, 204) '  blue landmark
    yellowLandmarkColor = RGB(255, 255, 0) ' yellow landmark
    refuelColor = RGB(50, 50, 50) ' refuel stations
    
    'draw the green landmarks
    pbxArena.DrawWidth = 4
    pbxArena.Line (xoffset + xf * 5.66, yoffset - yf * 11.95)-Step(-xf, 0), greenLandmarkColor '
    pbxArena.Line (xoffset + xf * -5.66, yoffset - yf * 11.95)-Step(xf, 0), greenLandmarkColor '
    pbxArena.Line (xoffset + xf * 5.95, yoffset - yf * 10.66)-Step(0, -yf), greenLandmarkColor '
    pbxArena.Line (xoffset + xf * -5.95, yoffset - yf * 10.66)-Step(0, -yf), greenLandmarkColor '
    pbxArena.DrawWidth = 1
    
    'draw the refuel stations
'    pbxArena.DrawWidth = 4
'    pbxArena.Line (xoffset + xf * 6, yoffset - yf * 10)-Step(-xf, 0), refuelColor '
'    pbxArena.Circle Step(0, 0), xf * 0.1, refuelColor
'    pbxArena.Line (xoffset + xf * -6, yoffset - yf * 10)-Step(xf, 0), refuelColor '
'    pbxArena.Circle Step(0, 0), xf * 0.1, refuelColor
'    pbxArena.DrawWidth = 1
    
    'Draw the golf ball pits
    pbxArena.FillStyle = 1
    pbxArena.Circle (xoffset + xf * 2, yoffset + yf * 4), xf * 1, orangeGBColor
    pbxArena.Circle (xoffset - xf * 2, yoffset + yf * 4), xf * 1, blueGBColor 'blue at (-2,-2)
    pbxArena.Circle (xoffset + xf * 2, yoffset + yf * 4), xf * 0.975, orangeGBColor
    pbxArena.Circle (xoffset - xf * 2, yoffset + yf * 4), xf * 0.975, blueGBColor 'blue at (-2,-2)
    pbxArena.Circle (xoffset + xf * 2, yoffset + yf * 4), xf * 0.95, orangeGBColor
    pbxArena.Circle (xoffset - xf * 2, yoffset + yf * 4), xf * 0.95, blueGBColor 'blue at (-2,-2)
    pbxArena.Circle (xoffset + xf * 2, yoffset + yf * 4), xf * 0.925, orangeGBColor
    pbxArena.Circle (xoffset - xf * 2, yoffset + yf * 4), xf * 0.925, blueGBColor 'blue at (-2,-2)
    pbxArena.Circle (xoffset + xf * 2, yoffset + yf * 4), xf * 0.9, orangeGBColor
    pbxArena.Circle (xoffset - xf * 2, yoffset + yf * 4), xf * 0.9, blueGBColor 'blue at (-2,-2)
    'Draw starting location
    pbxArena.DrawWidth = 2
    pbxArena.Line (xoffset + xf * -0.1, yoffset + yf * -0.1)-Step(xf * 0.2, yf * 0.2), blueLandmarkColor
    pbxArena.Line (xoffset + xf * -0.1, yoffset + yf * 0.1)-Step(xf * 0.2, yf * -0.2), blueLandmarkColor
    
    'Draw the lights
'    pbxArena.DrawWidth = 1
'    pbxArena.FillStyle = vbFSSolid
'    pbxArena.FillColor = lightColor
'    pbxArena.Circle (xoffset + xf * 5.95, yoffset - yf * 2), xf * 0.1, vbBlack, -0.001, -pi
'    pbxArena.Circle (xoffset + xf * -5.95, yoffset - yf * 2), xf * 0.1, vbBlack, -pi, -2 * pi
'    pbxArena.Circle (xoffset, yoffset - yf * 3.8), xf * 0.1, vbBlack, -pi / 2#, -3# * pi / 2#
'    pbxArena.Circle (xoffset, yoffset - yf * 4.2), xf * 0.1, vbBlack, -3# * pi / 2#, -pi / 2#
    'draw can line
'    pbxArena.DrawWidth = 2
'    pbxArena.Line (xoffset + xf * -2, yoffset - yf * 5)-Step(xf * 4, 0), canColor 'Can behind line
    

    ' draw possible golf ball locations
   ' pbxArena.Font.Size = 10
   ' pbxArena.Circle (xf * 7.5, yf * 8), xf * 0.1, QBColor(8)
   ' pbxArena.Print ("1")

    
    'pbxArena.Lin
'    For I = 0 To (4)
'        pbxArena.Line (xoffset + xf * gbAreaXs(I), yoffset - yf * gbAreaYs(I))-(xoffset + xf * gbAreaXe(I), yoffset - yf * gbAreaYs(I)), QBColor(5)
'        pbxArena.Line (xoffset + xf * gbAreaXe(I), yoffset - yf * gbAreaYs(I))-(xoffset + xf * gbAreaXe(I), yoffset - yf * gbAreaYe(I)), QBColor(5)
'        pbxArena.Line (xoffset + xf * gbAreaXe(I), yoffset - yf * gbAreaYe(I))-(xoffset + xf * gbAreaXs(I), yoffset - yf * gbAreaYe(I)), QBColor(5)
'        pbxArena.Line (xoffset + xf * gbAreaXs(I), yoffset - yf * gbAreaYe(I))-(xoffset + xf * gbAreaXs(I), yoffset - yf * gbAreaYs(I)), QBColor(5)
'    Next I
    
    'Draw Star Corner path points
    pbxArena.Circle (xoffset + xf * 0, yoffset - yf * -1), xf * 0.25, vbBlack
    lblStart.Left = xoffset + xf * 0.25
    lblStart.Top = yoffset + yf * 1.25
    pbxArena.Circle (xoffset + xf * -5, yoffset - yf * -3), xf * 0.25, vbBlack
    lblOne.Left = xoffset + xf * -4.75
    lblOne.Top = yoffset - yf * -3.25
    pbxArena.Circle (xoffset + xf * 3, yoffset - yf * 7), xf * 0.25, vbBlack
    lblTwo.Left = xoffset + xf * 3.25
    lblTwo.Top = yoffset - yf * 6.75
    pbxArena.Circle (xoffset + xf * -3, yoffset - yf * 7), xf * 0.25, vbBlack
    lblThree.Left = xoffset + xf * -2.75
    lblThree.Top = yoffset - yf * 6.75
    pbxArena.Circle (xoffset + xf * 5, yoffset - yf * -3), xf * 0.25, vbBlack
    lblFour.Left = xoffset + xf * 5.25
    lblFour.Top = yoffset - yf * -3.25
    pbxArena.Circle (xoffset + xf * 0, yoffset - yf * 11), xf * 0.25, vbBlack
    lblFive.Left = xoffset + xf * 0.25
    lblFive.Top = yoffset - yf * 10.75
    
    
    'save this as the default background
    pbxArena.Picture = pbxArena.Image

End Sub





Private Sub HScrollObstacle_Change()
    lblobstacle.Caption = HScrollObstacle.Value
End Sub

Private Sub Timer1_Timer()
    TimerTime = TimerTime + Timer1.Interval / 1000
    lblTime = "Time:     " + Format(TimerTime, "#0.00s")
End Sub



Private Sub VScrollBlue_Change()
'    lblBlue = 3 - VScrollBlue.Value
    lblBlue = VScrollBlue.Value
End Sub

Private Sub VScrollOrange_Change()
    'lblOrange = 3 - VScrollOrange.Value
    lblOrange = VScrollOrange.Value
End Sub
