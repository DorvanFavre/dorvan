import 'package:dorvan/application/providers/language_state_provider.dart';
import 'package:dorvan/domain/entity/language.dart';
import 'package:flutter/material.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';
import 'package:riverpod/riverpod.dart';
import 'package:enum_to_string/enum_to_string.dart';

class LanguageSelection extends StatelessWidget {
  @override
  Widget build(BuildContext context) {
    return Column(
        mainAxisAlignment: MainAxisAlignment.center,
        children: Language.values.map((language) {
          return LanguageTitle(
            language: language,
          );
        }).toList());
  }
}

class LanguageTitle extends ConsumerWidget {
  final Language? language;

  LanguageTitle({@required this.language});

  @override
  Widget build(BuildContext context, ScopedReader watch) {
    final currentLanguage = watch(languageStateProvider);

    return GestureDetector(
        onTap: () {
          currentLanguage.state = language!;
        },
        child: Padding(
          padding: const EdgeInsets.symmetric(vertical: 10),
          child: Text(
            EnumToString.convertToString(language),
            style: TextStyle(
                color: currentLanguage.state == language
                    ? Colors.white
                    : Colors.white38),
          ),
        ));
  }
}
