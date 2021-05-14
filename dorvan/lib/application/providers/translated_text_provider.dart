import 'package:dorvan/application/constants/translated_text.dart';
import 'package:dorvan/application/providers/language_state_provider.dart';
import 'package:dorvan/domain/entity/language.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

final translatedTextProvider = Provider<Map<String, String>>((ref) {
  final language = ref.watch(languageStateProvider).state;
  return kTranslatedText[language] ?? kTranslatedText[Language.EN]!;
});
