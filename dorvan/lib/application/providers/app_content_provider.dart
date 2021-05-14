import 'package:dorvan/domain/entity/article.dart';
import 'package:dorvan/domain/entity/chapter.dart';
import 'package:dorvan/domain/entity/contact.dart';
import 'package:dorvan/domain/entity/intro.dart';
import 'package:dorvan/domain/entity/page_content.dart';
import 'package:dorvan/domain/entity/picture.dart';
import 'package:flutter_riverpod/flutter_riverpod.dart';

final appContentProvider = Provider<List<PageContent>>((ref) {
  return [
    Intro(
      "chapter_intro_title",
      "chapter_intro_description",
      '00',
    ),
    Picture('02_iphone12.jpg', 'Vans', 'salut'),
    // Entrepreneur
    Chapter('entrepreneur_title', "entrepreneur_description", '10', [
      Article('entrepreneur_entrepreneur_title',
          'entrepreneur_entrepreneur_description', '10'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('entrepreneur_parlezmoi_title',
          'entrepreneur_parlezmoi_description', '11'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('entrepreneur_tirerprofis_title',
          'entrepreneur_tirerprofis_description', '12'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('entrepreneur_boutenbout_title',
          'entrepreneur_boutenbout_description', '13'),
      Picture('vans.jpg', 'Vans', 'salut'),
    ]),
    Picture('vans.jpg', 'Vans', 'salut'),
    // Architect
    Chapter('architect_title', "architect_description", '20', [
      Article(
          'architect_architect_title', 'architect_architect_description', '20'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('architect_nature_title', 'architect_nature_description', '21'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('architext_plannifier_title', 'architext_plannifier_description',
          '21'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('architect_deroulement_title',
          'architect_deroulement_description', '21'),
      Picture('vans.jpg', 'Vans', 'salut'),
    ]),
    Picture('vans.jpg', 'Vans', 'salut'),
    Chapter('entrepreneur_title', "entrepreneur_description", '30', [
      Article('entrepreneur_entrepreneur_title',
          'entrepreneur_entrepreneur_description', '30'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('entrepreneur_parlezmoi_title',
          'entrepreneur_parlezmoi_description', '31'),
    ]),
    Picture('vans.jpg', 'Vans', 'salut'),
    Chapter('entrepreneur_title', "entrepreneur_description", '04', [
      Article('entrepreneur_entrepreneur_title',
          'entrepreneur_entrepreneur_description', '01'),
      Picture('vans.jpg', 'Vans', 'salut'),
      Article('entrepreneur_parlezmoi_title',
          'entrepreneur_parlezmoi_description', '02'),
    ]),
    Picture('vans.jpg', 'Vans', 'salut'),
    Contact(),
    Picture('vans.jpg', 'Vans', 'salut'),
  ];
});
