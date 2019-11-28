int music_tone[50][3];

void clean_tones() {
	memset(music_tone, -1, 50 * 3 * sizeof(int));
}

void sound_attack() { // Switch to Attack
//	clean_tones();
//
//	music_tone[0][0] = 659;
//	music_tone[0][1] = 400;
//	music_tone[0][2] = 5;
//	music_tone[1][0] = 587;
//	music_tone[1][1] = 400;
//	music_tone[1][2] = 5;
//	music_tone[2][0] = 523;
//	music_tone[2][1] = 400;
//	music_tone[2][2] = 5;
//
//	BT_play_tone_sequence(music_tone);
}

void sound_kick() { // Kick the Ball
//	clean_tones();
//
//	music_tone[0][0] = 523;
//	music_tone[0][1] = 400;
//	music_tone[0][2] = 5;
//	music_tone[1][0] = 659;
//	music_tone[1][1] = 400;
//	music_tone[1][2] = 5;
//	music_tone[2][0] = 523;
//	music_tone[2][1] = 400;
//	music_tone[2][2] = 5;
//
//	BT_play_tone_sequence(music_tone);
}

void sound_defend() { // Switch to Defense
//	clean_tones();
//
//	music_tone[0][0] = 523;
//	music_tone[0][1] = 400;
//	music_tone[0][2] = 5;
//	music_tone[1][0] = 587;
//	music_tone[1][1] = 400;
//	music_tone[1][2] = 5;
//	music_tone[2][0] = 659;
//	music_tone[2][1] = 400;
//	music_tone[2][2] = 5;
//
//	BT_play_tone_sequence(music_tone);
}

void play_white_tone() {
	clean_tones();

	music_tone[0][0] = 523;
	music_tone[0][1] = 400;
	music_tone[0][2] = 5;
	music_tone[1][0] = 523;
	music_tone[1][1] = 400;
	music_tone[1][2] = 5;
	music_tone[2][0] = 523;
	music_tone[2][1] = 400;
	music_tone[2][2] = 5;

	BT_play_tone_sequence(music_tone);
}
