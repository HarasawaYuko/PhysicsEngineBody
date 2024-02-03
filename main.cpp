#include "DxLib.h"
#include "SceneMgr.h"
#include "FpsControl.h"
#include "Share.h"

//�萔
//�E�B���h�E�̏����ʒu
#define WIN_POS_X 0
#define WIN_POS_Y 0
#define WIN_SIZE_X 800
#define WIN_SIZE_Y 600

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance, LPSTR lpCmdLine, int nCmdShow)
{
    // �E�B���h�E���[�h�ɐݒ�
    ChangeWindowMode(TRUE);
    SetWaitVSyncFlag(FALSE);
    // DX���C�u��������������
    if (DxLib_Init() < 0) {
        //�G���[�Ȃ�I������
        return -1;
    }
    //�`���𗠉�ʂɂ���
    SetDrawScreen(DX_SCREEN_BACK);
    //��ʓ��̐ݒ�
    SetWindowText("Physics Engine");
    SetGraphMode(WIN_SIZE_X , WIN_SIZE_Y , 32);
    SetWindowInitPosition(WIN_POS_X , WIN_POS_Y);
    SetBackgroundColor(255 , 255 , 255);
    ChangeFontType(DX_FONTTYPE_ANTIALIASING);
    ChangeFont("Meiryo UI");

    //x�{�^���ŏI�����Ȃ�
    SetWindowUserCloseEnableFlag(false);

    SceneMgr sceneMgr;
    FpsControl fpsControl;

    //������
    sceneMgr.Initialize();
    Share::initialize();

    // while(����ʂ�\��ʂɔ��f, ���b�Z�[�W����, ��ʃN���A)
    while ( ProcessMessage() == 0 && ClearDrawScreen() == 0) {
        fpsControl.Update();
        sceneMgr.Update();
        //fpsControl.Draw();
        sceneMgr.Draw();
        ScreenFlip();
        fpsControl.Wait();
        if (sceneMgr.fin) break;
    }
    sceneMgr.Finalize();
    DxLib_End();    // DX���C�u�����I������
    return 0;
}